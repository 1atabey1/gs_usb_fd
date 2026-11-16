"""A pyusb-level stand-in for a gs_usb FD device (two channels, 80 MHz)."""

from __future__ import annotations

import queue
import struct
import time

import usb.core

from python_can_gs_usb_fd import protocol as p

FEATURES = (p.FEATURE_LISTEN_ONLY | p.FEATURE_LOOP_BACK | p.FEATURE_ONE_SHOT | p.FEATURE_HW_TIMESTAMP
            | p.FEATURE_FD | p.FEATURE_BT_CONST_EXT | p.FEATURE_BERR_REPORTING | p.FEATURE_GET_STATE)


class _Ep:
    def __init__(self, addr):
        self.bEndpointAddress = addr
        self.bmAttributes = 0x02  # bulk


class _Config:
    def __getitem__(self, key):
        return [_Ep(0x81), _Ep(0x02)]


class FakeGsUsb:
    def __init__(self, serial="FAKE0001", channels=2):
        self.serial_number = serial
        self.bus, self.address = 1, 7
        self.channels = channels
        self.requests: list[tuple[int, int, bytes]] = []  # (bRequest, wValue, data)
        self.mode: dict[int, tuple[int, int]] = {}
        self.bittiming: dict[int, tuple] = {}
        self.data_bittiming: dict[int, tuple] = {}
        self.written: list[bytes] = []
        self.inq: queue.Queue = queue.Queue()
        self.state = (p.STATE_ERROR_ACTIVE, 0, 0)
        self.ts = 1000

    # pyusb device API used by the plugin
    def is_kernel_driver_active(self, iface):
        return False

    def get_active_configuration(self):
        return _Config()

    def ctrl_transfer(self, req_type, req, value, index, data_or_len, timeout):
        if req_type == p.REQ_OUT:
            data = bytes(data_or_len)
            self.requests.append((req, value, data))
            if req == p.BREQ_MODE:
                self.mode[value] = struct.unpack("<II", data)
            elif req == p.BREQ_BITTIMING:
                self.bittiming[value] = struct.unpack("<5I", data)
            elif req == p.BREQ_DATA_BITTIMING:
                self.data_bittiming[value] = struct.unpack("<5I", data)
            return len(data)
        if req == p.BREQ_DEVICE_CONFIG:
            return bytes([0, 0, 0, self.channels - 1]) + struct.pack("<II", 2, 1)
        if req in (p.BREQ_BT_CONST, p.BREQ_BT_CONST_EXT):
            raw = struct.pack("<18I", FEATURES, 80_000_000, 1, 256, 1, 128, 128, 1, 512, 1,
                              1, 32, 1, 16, 16, 1, 32, 1)
            return raw[:data_or_len]
        if req == p.BREQ_GET_STATE:
            return struct.pack("<3I", *self.state)
        raise usb.core.USBError(f"unexpected request {req}")

    def write(self, ep, data, timeout):
        data = bytes(data)
        self.written.append(data)
        echo_id, can_id, dlc, ch, flags, _ = p.HEADER.unpack_from(data)
        payload = data[p.HEADER_LEN:]
        self.ts += 100
        ts = struct.pack("<I", self.ts)
        # echo (the TX confirmation), then the frame itself if in loopback
        self.inq.put(data + ts)
        mode, mflags = self.mode.get(ch, (0, 0))
        if mflags & p.FEATURE_LOOP_BACK:
            self.inq.put(p.HEADER.pack(p.ECHO_ID_RX, can_id, dlc, ch, flags, 0) + payload + ts)
        return len(data)

    def read(self, ep, size, timeout):
        try:
            return self.inq.get(timeout=timeout / 1000)
        except queue.Empty:
            raise usb.core.USBTimeoutError("timeout") from None

    # test helpers
    def inject(self, ch, can_id, data, fd=False, brs=False, flags=0):
        self.ts += 100
        dlc = p.len_to_dlc(len(data)) if fd else len(data)
        fl = flags | (p.FLAG_FD if fd else 0) | (p.FLAG_BRS if brs else 0)
        body = bytes(data).ljust(64 if fd else 8, b"\0")
        self.inq.put(p.HEADER.pack(p.ECHO_ID_RX, can_id, dlc, ch, fl, 0) + body + struct.pack("<I", self.ts))
        time.sleep(0)
