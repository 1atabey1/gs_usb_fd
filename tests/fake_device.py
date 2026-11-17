"""A pyusb-level stand-in for a gs_usb FD device (80 MHz).

By default one gs_usb interface (0) with two channels. `interfaces` gives
other layouts, e.g. ((0, 1), (4, 1)) for the tricore-debug-machine probe's
split layout. The recorded state (mode, bittiming, ...) is keyed by the
device's channel number, counted across the interfaces.
"""

from __future__ import annotations

import queue
import struct
import time

import usb.core

from python_can_gs_usb_fd import protocol as p

FEATURES = (p.FEATURE_LISTEN_ONLY | p.FEATURE_LOOP_BACK | p.FEATURE_ONE_SHOT | p.FEATURE_HW_TIMESTAMP
            | p.FEATURE_FD | p.FEATURE_BT_CONST_EXT | p.FEATURE_BERR_REPORTING | p.FEATURE_GET_STATE)

# endpoints per interface number: (IN, OUT)
EPS = {0: (0x81, 0x02), 4: (0x86, 0x05)}


class _Ep:
    def __init__(self, addr):
        self.bEndpointAddress = addr
        self.bmAttributes = 0x02  # bulk


class _Intf:
    def __init__(self, num, cls=(0xFF, 0xFF, 0xFF)):
        self.bInterfaceNumber = num
        self.bInterfaceClass, self.bInterfaceSubClass, self.bInterfaceProtocol = cls

    def __iter__(self):
        return iter(_Ep(a) for a in EPS[self.bInterfaceNumber])


class _Config:
    def __init__(self, nums):
        self.nums = nums

    def __iter__(self):
        # a non-gs interface in between, as the probe's DAP interface
        return iter([_Intf(n) for n in self.nums] + [_Intf(3, (0xFF, 0x00, 0x00))])

    def __getitem__(self, key):
        return _Intf(key[0])


class FakeGsUsb:
    def __init__(self, serial="FAKE0001", channels=2, interfaces=None):
        self.serial_number = serial
        self.bus, self.address = 1, 7
        self.interfaces = tuple(interfaces or ((0, channels),))
        self.channels = sum(n for _, n in self.interfaces)
        self.requests: list[tuple[int, int, bytes]] = []  # (bRequest, device channel, data)
        self.mode: dict[int, tuple[int, int]] = {}
        self.bittiming: dict[int, tuple] = {}
        self.data_bittiming: dict[int, tuple] = {}
        self.written: list[bytes] = []
        self.inq = {i: queue.Queue() for i, _ in self.interfaces}
        self.state = (p.STATE_ERROR_ACTIVE, 0, 0)
        self.ts = 1000
        self.busy: set[int] = set()  # interfaces "another process" holds

    def _base(self, iface):
        base = 0
        for i, n in self.interfaces:
            if i == iface:
                return base, n
            base += n
        raise usb.core.USBError(f"no interface {iface}")

    def _iface_of_ep(self, ep):
        return next(i for i, eps in EPS.items() if ep in eps)

    def locate(self, ch):
        """device channel -> (interface, local channel)"""
        base = 0
        for i, n in self.interfaces:
            if ch < base + n:
                return i, ch - base
            base += n
        raise ValueError(ch)

    # pyusb device API used by the plugin
    def is_kernel_driver_active(self, iface):
        return False

    def claim(self, iface):
        if iface in self.busy:
            raise usb.core.USBError("Access denied (insufficient permissions)", errno=13)

    def get_active_configuration(self):
        return _Config([i for i, _ in self.interfaces])

    def ctrl_transfer(self, req_type, req, value, index, data_or_len, timeout):
        base, n = self._base(index)
        if req_type == p.REQ_OUT:
            data = bytes(data_or_len)
            ch = base + value
            self.requests.append((req, ch, data))
            if req == p.BREQ_MODE:
                self.mode[ch] = struct.unpack("<II", data)
            elif req == p.BREQ_BITTIMING:
                self.bittiming[ch] = struct.unpack("<5I", data)
            elif req == p.BREQ_DATA_BITTIMING:
                self.data_bittiming[ch] = struct.unpack("<5I", data)
            return len(data)
        if req == p.BREQ_DEVICE_CONFIG:
            return bytes([0, 0, 0, n - 1]) + struct.pack("<II", 2, 1)
        if req in (p.BREQ_BT_CONST, p.BREQ_BT_CONST_EXT):
            raw = struct.pack("<18I", FEATURES, 80_000_000, 1, 256, 1, 128, 128, 1, 512, 1,
                              1, 32, 1, 16, 16, 1, 32, 1)
            return raw[:data_or_len]
        if req == p.BREQ_GET_STATE:
            return struct.pack("<3I", *self.state)
        raise usb.core.USBError(f"unexpected request {req}")

    def write(self, ep, data, timeout):
        iface = self._iface_of_ep(ep)
        base, _ = self._base(iface)
        data = bytes(data)
        self.written.append(data)
        echo_id, can_id, dlc, ch, flags, _ = p.HEADER.unpack_from(data)
        payload = data[p.HEADER_LEN:]
        self.ts += 100
        ts = struct.pack("<I", self.ts)
        # echo (the TX confirmation), then the frame itself if in loopback
        self.inq[iface].put(data + ts)
        mode, mflags = self.mode.get(base + ch, (0, 0))
        if mflags & p.FEATURE_LOOP_BACK:
            self.inq[iface].put(p.HEADER.pack(p.ECHO_ID_RX, can_id, dlc, ch, flags, 0) + payload + ts)
        return len(data)

    def read(self, ep, size, timeout):
        try:
            return self.inq[self._iface_of_ep(ep)].get(timeout=timeout / 1000)
        except queue.Empty:
            raise usb.core.USBTimeoutError("timeout") from None

    # test helpers
    def inject(self, ch, can_id, data, fd=False, brs=False, flags=0):
        """A received frame on device channel ch."""
        iface, local = self.locate(ch)
        self.ts += 100
        dlc = p.len_to_dlc(len(data)) if fd else len(data)
        fl = flags | (p.FLAG_FD if fd else 0) | (p.FLAG_BRS if brs else 0)
        body = bytes(data).ljust(64 if fd else 8, b"\0")
        self.inq[iface].put(p.HEADER.pack(p.ECHO_ID_RX, can_id, dlc, local, fl, 0) + body
                            + struct.pack("<I", self.ts))
        time.sleep(0)
