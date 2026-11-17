"""gs_usb devices, their interfaces ("ports"), and the channels on them.

A gs_usb interface carries one or more CAN channels on one bulk IN and one
bulk OUT endpoint, so a bus per channel that read the IN endpoint on its own
would steal the other channels' frames. Each interface is therefore opened
once per process (keyed by device serial and interface number), one reader
thread drains its IN endpoint and routes frames to per-channel queues, and
it is closed when its last bus shuts down.

A device may have several gs_usb interfaces: the tricore-debug-machine probe
in its "split" USB layout puts CAN1 on interface 0 and CAN2 on interface 4,
so two programs can each have one channel (Windows' WinUSB gives out one
handle per interface). Channels are numbered across a device's gs_usb
interfaces in interface order: channel N of the device is local channel
N - base on the interface that holds it. Callers see the same channel
numbers in either layout.

Windows: each interface is opened alone through WinUSB (found by its
DeviceInterfaceGUID), never the whole device as libusb would, so other
programs keep the device's other interfaces. Elsewhere (or with
GS_USB_FD_BACKEND=libusb): pyusb and libusb-1.0, where a process can claim
some interfaces of a device while another process claims others.
"""

from __future__ import annotations

import os
import queue
import struct
import threading
import time
from dataclasses import dataclass, field

import usb.core
import usb.util

from . import _winusb
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

# DeviceInterfaceGUIDs that gs_usb firmware registers for WinUSB (MS OS
# descriptors): candleLight's, which the tricore-debug-machine probe uses too.
WINUSB_GUIDS = ("{c15b4308-04d3-11e6-b3ea-6057189e6443}",)

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
    """pyusb devices with a known VID:PID (the libusb path)."""
    backend = usb_backend()
    found = []
    for vid, pid in KNOWN_IDS:
        found.extend(usb.core.find(find_all=True, idVendor=vid, idProduct=pid, backend=backend) or [])
    return found


def gs_interfaces(dev) -> list[int]:
    """The gs_usb interface numbers of a pyusb device: vendor class with
    subclass and protocol 0xFF, as candleLight's interface is; [0] if none
    says so (gs_usb devices that describe their interface otherwise)."""
    try:
        cfg = dev.get_active_configuration()
        found = sorted({i.bInterfaceNumber for i in cfg
                        if (i.bInterfaceClass, i.bInterfaceSubClass, i.bInterfaceProtocol) == (0xFF, 0xFF, 0xFF)})
    except (usb.core.USBError, NotImplementedError, TypeError, AttributeError):
        found = []
    return found or [0]


def _use_winusb() -> bool:
    return _winusb.AVAILABLE and os.environ.get("GS_USB_FD_BACKEND", "").lower() != "libusb"


@dataclass
class PortRef:
    """A gs_usb interface found on the bus, not opened yet."""

    serial: str
    interface: int
    target: object = field(repr=False)  # pyusb Device, or _winusb.InterfacePath

    @property
    def key(self) -> tuple[str, int]:
        return self.serial, self.interface


def find_ports() -> list[PortRef]:
    """Every gs_usb interface of every device found, by serial then
    interface number. Windows: through WinUSB when any device registered a
    known GUID, else (other drivers) through libusb."""
    ports: list[PortRef] = []
    if _use_winusb():
        for guid in WINUSB_GUIDS:
            for i in _winusb.list_interfaces(guid):
                if (i.vid, i.pid) in KNOWN_IDS:
                    ports.append(PortRef(i.serial, max(i.interface, 0), i))
        if ports:
            return sorted(ports, key=lambda x: x.key)
    for d in find_usb_devices():
        serial = _serial(d)
        ports.extend(PortRef(serial, n, d) for n in gs_interfaces(d))
    return sorted(ports, key=lambda x: x.key)


def ports_by_serial(ports: list[PortRef]) -> dict[str, list[PortRef]]:
    out: dict[str, list[PortRef]] = {}
    for x in ports:
        out.setdefault(x.serial, []).append(x)
    return out


class InterfaceBusyError(LookupError):
    """The gs_usb interface holding a channel is open in another program."""


@dataclass
class _Channel:
    queue: queue.Queue
    hw_timestamp: bool = True


# pyusb devices shared by several open interfaces in this process: the handle
# closes when the last one is released (dispose_resources releases them all)
_usb_refs: dict[int, int] = {}
_usb_refs_lock = threading.Lock()


class GsUsbDevice:
    """One open gs_usb interface (port) of a device; channels are its local
    ones (0 .. channel_count - 1)."""

    _registry: dict[tuple[str, int], GsUsbDevice] = {}
    _registry_lock = threading.Lock()

    # ---- process-wide sharing
    @classmethod
    def acquire_port(cls, ref: PortRef) -> GsUsbDevice:
        """The open port for ref, opening it on first use. Pair with
        release(). Caller holds _registry_lock."""
        dev = cls._registry.get(ref.key)
        if dev is not None:
            dev._refs += 1
            return dev
        dev = cls(ref)
        cls._registry[ref.key] = dev
        return dev

    @classmethod
    def acquire_channel(cls, serial: str | None, channel: int) -> tuple[GsUsbDevice, int]:
        """The open port holding the device's channel `channel` (counted
        across its gs_usb interfaces) and the channel's number on that port.
        serial None: the first device found. Pair with release().

        A port that another process has open cannot be asked for its channel
        count; it counts as one channel (as each interface of the probe's
        split layout has)."""
        with cls._registry_lock:
            groups = ports_by_serial(find_ports())
            if serial is None:
                if not groups:
                    raise LookupError("no gs_usb device found")
                serial = next(iter(groups))
            refs = groups.get(serial)
            if not refs:
                raise LookupError(f"no gs_usb device found with serial {serial}")
            base = 0
            for ref in refs:
                try:
                    dev = cls.acquire_port(ref)
                except usb.core.USBError as e:
                    if base == channel:
                        raise InterfaceBusyError(
                            f"channel {channel} of {serial} (interface {ref.interface}) is busy: "
                            f"another program has it open") from e
                    base += 1
                    continue
                n = dev.channel_count
                if channel < base + n:
                    return dev, channel - base
                dev._release_locked()
                base += n
            raise LookupError(f"channel {channel} out of range, {serial} has {base}")

    def release(self) -> None:
        with self._registry_lock:
            self._release_locked()

    def _release_locked(self) -> None:
        self._refs -= 1
        if self._refs > 0:
            return
        self._registry.pop(self.key, None)
        self._close()

    # ---- open / close
    def __init__(self, ref: PortRef):
        self.serial = ref.serial
        self.iface = ref.interface
        self.key = ref.key
        self._winusb = isinstance(ref.target, _winusb.InterfacePath)
        self._refs = 1
        self._ctrl_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._channels: dict[int, _Channel] = {}
        self._channels_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None
        self._reader_error: BaseException | None = None
        self._clock = p.TimestampUnwrapper(time.time())

        if self._winusb:
            self.usb = _winusb.WinUsbInterface(ref.target)
        else:
            self.usb = ref.target
            try:
                if self.usb.is_kernel_driver_active(self.iface):
                    self.usb.detach_kernel_driver(self.iface)
            except (NotImplementedError, usb.core.USBError):
                pass
            usb.util.claim_interface(self.usb, self.iface)
            with _usb_refs_lock:
                _usb_refs[id(self.usb)] = _usb_refs.get(id(self.usb), 0) + 1
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
            self._close_usb()
            raise

    def _find_endpoints(self) -> tuple[int, int]:
        ep_in = ep_out = None
        if self._winusb:
            eps = [(addr, kind) for addr, kind, _mps in self.usb.pipes()]
        else:
            intf = self.usb.get_active_configuration()[(self.iface, 0)]
            eps = [(ep.bEndpointAddress, usb.util.endpoint_type(ep.bmAttributes)) for ep in intf]
        for addr, kind in eps:
            if kind != usb.util.ENDPOINT_TYPE_BULK:
                continue
            if usb.util.endpoint_direction(addr) == usb.util.ENDPOINT_IN:
                ep_in = ep_in or addr
            else:
                ep_out = ep_out or addr
        if ep_in is None or ep_out is None:
            raise LookupError(f"interface {self.iface} has no bulk IN/OUT endpoint pair")
        return ep_in, ep_out

    def _close_usb(self) -> None:
        if self._winusb:
            self.usb.close()
            return
        try:
            usb.util.release_interface(self.usb, self.iface)
        except usb.core.USBError:
            pass
        with _usb_refs_lock:
            left = _usb_refs.get(id(self.usb), 1) - 1
            if left > 0:
                _usb_refs[id(self.usb)] = left
            else:
                _usb_refs.pop(id(self.usb), None)
        if left <= 0:
            usb.util.dispose_resources(self.usb)

    def _close(self) -> None:
        self._stop.set()
        if self._reader is not None:
            self._reader.join(timeout=2 * READ_TIMEOUT_MS / 1000 + 1)
        self._close_usb()

    # ---- control requests
    def ctrl_out(self, req: int, data: bytes, value: int = 0) -> None:
        with self._ctrl_lock:
            self.usb.ctrl_transfer(p.REQ_OUT, req, value, self.iface, data, 1000)

    def ctrl_in(self, req: int, length: int, value: int = 0) -> bytes:
        with self._ctrl_lock:
            return bytes(self.usb.ctrl_transfer(p.REQ_IN, req, value, self.iface, length, 1000))

    # ---- channels
    def open_channel(self, ch: int, hw_timestamp: bool) -> queue.Queue:
        with self._channels_lock:
            if ch in self._channels:
                raise RuntimeError(f"channel {ch} of {self.serial} interface {self.iface} is already open")
            c = _Channel(queue.Queue(QUEUE_MAX), hw_timestamp)
            self._channels[ch] = c
        if self._reader is None:
            self._reader = threading.Thread(target=self._read_loop,
                                            name=f"gs_usb_fd {self.serial}/{self.iface}", daemon=True)
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
