"""One gs_usb device, shared by every bus open on its channels.

All channels of a gs_usb device share one bulk IN endpoint, so a bus per
channel that read it on its own would steal the other channels' frames. A
device is opened once per process (keyed by its serial number), one reader
thread drains the IN endpoint and routes frames to per-channel queues, and
the device is closed when its last bus shuts down.
"""

from __future__ import annotations

import queue
import struct
import threading
import time
from dataclasses import dataclass

import usb.core
import usb.util

from . import protocol as p

# gs_usb devices this interface looks for (the gs_usb Python package's list).
# Only those that advertise CAN FD can be opened.
KNOWN_IDS = (
    (0x1D50, 0x606F),  # candleLight / gs_usb (also the tricore-debug-machine probe)
    (0x1209, 0x2323),  # candleLight (pid.codes)
    (0x1CD2, 0x606F),  # CES CANext FD
    (0x16D0, 0x10B8),  # ABE CANdebugger FD
    (0x1209, 0xCA01),  # CANnectivity
)

IFACE = 0
READ_TIMEOUT_MS = 100
QUEUE_MAX = 100_000  # per channel, frames; the oldest are dropped beyond it


def usb_backend():
    """A libusb-1.0 backend: the system's, else the DLL bundled by
    libusb-package. Never pyusb's default order, which falls back to libusb0:
    libusb-win32 leaves libusb0.dll in System32, and libusb0 cannot see
    WinUSB devices (the device then looks absent)."""
    import usb.backend.libusb1

    backend = usb.backend.libusb1.get_backend()
    if backend is not None:
        return backend
    try:
        import libusb_package
    except ImportError:
        raise usb.core.NoBackendError("no libusb-1.0 (pip install libusb-package)") from None
    return libusb_package.get_libusb1_backend()


def _serial(dev) -> str:
    """The device's serial number (bus/address if it has none). Reading it
    opens a libusb handle; it is released again, since a second handle on the
    same device (a later enumeration) cannot read strings on Windows."""
    if getattr(dev, "_gs_usb_fd_serial", None):
        return dev._gs_usb_fd_serial
    try:
        s = dev.serial_number or f"bus{dev.bus}-addr{dev.address}"
    except (usb.core.USBError, ValueError, NotImplementedError):
        s = f"bus{dev.bus}-addr{dev.address}"
    try:
        dev._gs_usb_fd_serial = s
        usb.util.dispose_resources(dev)
    except (AttributeError, usb.core.USBError):
        pass
    return s


def find_usb_devices() -> list:
    backend = usb_backend()
    found = []
    for vid, pid in KNOWN_IDS:
        found.extend(usb.core.find(find_all=True, idVendor=vid, idProduct=pid, backend=backend) or [])
    return found


@dataclass
class _Channel:
    queue: queue.Queue
    hw_timestamp: bool = True


class GsUsbDevice:
    _registry: dict[str, GsUsbDevice] = {}
    _registry_lock = threading.Lock()

    # ---- process-wide sharing
    @classmethod
    def acquire(cls, serial: str | None, usb_dev=None) -> GsUsbDevice:
        """The open device with this serial (the first one found if None),
        opening it on first use. `usb_dev`: an already enumerated device with
        that serial (spares a second enumeration). Pair with release()."""
        with cls._registry_lock:
            if serial is not None and serial in cls._registry:
                dev = cls._registry[serial]
                dev._refs += 1
                return dev
            if serial is None and cls._registry:
                dev = next(iter(cls._registry.values()))
                dev._refs += 1
                return dev
            usb_devs = [usb_dev] if usb_dev is not None else find_usb_devices()
            if serial is not None:
                usb_devs = [d for d in usb_devs if _serial(d) == serial]
            if not usb_devs:
                raise LookupError("no gs_usb device found" + (f" with serial {serial}" if serial else ""))
            dev = cls(usb_devs[0])
            cls._registry[dev.serial] = dev
            return dev

    def release(self) -> None:
        with self._registry_lock:
            self._refs -= 1
            if self._refs > 0:
                return
            self._registry.pop(self.serial, None)
        self._close()

    # ---- open / close
    def __init__(self, usb_dev):
        self.usb = usb_dev
        self.serial = _serial(usb_dev)
        self._refs = 1
        self._ctrl_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._channels: dict[int, _Channel] = {}
        self._channels_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None
        self._reader_error: BaseException | None = None
        self._clock = p.TimestampUnwrapper(time.time())

        try:
            if self.usb.is_kernel_driver_active(IFACE):
                self.usb.detach_kernel_driver(IFACE)
        except (NotImplementedError, usb.core.USBError):
            pass
        usb.util.claim_interface(self.usb, IFACE)
        try:
            self.ep_in, self.ep_out = self._find_endpoints()
            self.ctrl_out(p.BREQ_HOST_FORMAT, struct.pack("<I", 0x0000BEEF), value=1)
            cfg = self.ctrl_in(p.BREQ_DEVICE_CONFIG, 12, value=1)
            self.channel_count = cfg[3] + 1
            self.sw_version, self.hw_version = struct.unpack_from("<II", cfg, 4)
            self.bt_const = p.BitTimingConst.from_bytes(self.ctrl_in(p.BREQ_BT_CONST, 40, value=0))
            if self.bt_const.feature & p.FEATURE_BT_CONST_EXT:
                self.bt_const = p.BitTimingConst.from_bytes(self.ctrl_in(p.BREQ_BT_CONST_EXT, 72, value=0))
        except Exception:
            usb.util.release_interface(self.usb, IFACE)
            usb.util.dispose_resources(self.usb)
            raise

    def _find_endpoints(self) -> tuple[int, int]:
        intf = self.usb.get_active_configuration()[(IFACE, 0)]
        ep_in = ep_out = None
        for ep in intf:
            if usb.util.endpoint_type(ep.bmAttributes) != usb.util.ENDPOINT_TYPE_BULK:
                continue
            if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN:
                ep_in = ep_in or ep.bEndpointAddress
            else:
                ep_out = ep_out or ep.bEndpointAddress
        if ep_in is None or ep_out is None:
            raise LookupError(f"interface {IFACE} has no bulk IN/OUT endpoint pair")
        return ep_in, ep_out

    def _close(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=2 * READ_TIMEOUT_MS / 1000 + 1)
        try:
            usb.util.release_interface(self.usb, IFACE)
        except usb.core.USBError:
            pass
        usb.util.dispose_resources(self.usb)

    # ---- control requests
    def ctrl_out(self, req: int, data: bytes, value: int = 0) -> None:
        with self._ctrl_lock:
            self.usb.ctrl_transfer(p.REQ_OUT, req, value, IFACE, data, 1000)

    def ctrl_in(self, req: int, length: int, value: int = 0) -> bytes:
        with self._ctrl_lock:
            return bytes(self.usb.ctrl_transfer(p.REQ_IN, req, value, IFACE, length, 1000))

    # ---- channels
    def open_channel(self, ch: int, hw_timestamp: bool) -> queue.Queue:
        with self._channels_lock:
            if ch in self._channels:
                raise RuntimeError(f"channel {ch} of {self.serial} is already open")
            c = _Channel(queue.Queue(QUEUE_MAX), hw_timestamp)
            self._channels[ch] = c
        if self._reader is None:
            self._reader = threading.Thread(target=self._read_loop, name=f"gs_usb_fd {self.serial}",
                                            daemon=True)
            self._reader.start()
        return c.queue

    def close_channel(self, ch: int) -> None:
        with self._channels_lock:
            self._channels.pop(ch, None)

    def reader_error(self) -> BaseException | None:
        return self._reader_error

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                raw = bytes(self.usb.read(self.ep_in, 512, READ_TIMEOUT_MS))
            except usb.core.USBTimeoutError:
                continue
            except usb.core.USBError as e:
                if self._stop.is_set():
                    return
                self._reader_error = e
                return
            now = time.time()
            with self._channels_lock:
                channels = dict(self._channels)
            # Every open channel was started with the same timestamp setting.
            hw_ts = next(iter(channels.values())).hw_timestamp if channels else True
            for f in p.unpack_frames(raw, hw_ts):
                c = channels.get(f.channel)
                if c is None:
                    continue
                ts = self._clock(f.timestamp_us, now) if f.timestamp_us is not None else now
                try:
                    c.queue.put_nowait((ts, f))
                except queue.Full:
                    try:
                        c.queue.get_nowait()
                    except queue.Empty:
                        pass
                    c.queue.put_nowait((ts, f))

    # ---- transmit
    def write(self, frame: bytes, timeout_ms: int) -> None:
        with self._write_lock:
            self.usb.write(self.ep_out, frame, timeout_ms)
