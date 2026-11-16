"""gs_usb wire format (the Linux gs_usb host protocol with CAN FD).

Reference: the probe's firmware, Appli/usb/gs_usb.c, which follows the Linux
driver (drivers/net/can/usb/gs_usb.c) and candleLight_fw.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

# bRequest (vendor, recipient interface; wValue = channel, wIndex = interface)
BREQ_HOST_FORMAT = 0
BREQ_BITTIMING = 1
BREQ_MODE = 2
BREQ_BERR = 3
BREQ_BT_CONST = 4
BREQ_DEVICE_CONFIG = 5
BREQ_TIMESTAMP = 6
BREQ_IDENTIFY = 7
BREQ_DATA_BITTIMING = 10
BREQ_BT_CONST_EXT = 11
BREQ_GET_STATE = 14

REQ_OUT = 0x41  # host to device, vendor, interface
REQ_IN = 0xC1  # device to host, vendor, interface

MODE_RESET = 0
MODE_START = 1

# Feature bits (BT_CONST) double as mode flags (MODE).
FEATURE_LISTEN_ONLY = 1 << 0
FEATURE_LOOP_BACK = 1 << 1
FEATURE_TRIPLE_SAMPLE = 1 << 2
FEATURE_ONE_SHOT = 1 << 3
FEATURE_HW_TIMESTAMP = 1 << 4
FEATURE_IDENTIFY = 1 << 5
FEATURE_FD = 1 << 8
FEATURE_BT_CONST_EXT = 1 << 10
FEATURE_BERR_REPORTING = 1 << 12
FEATURE_GET_STATE = 1 << 13

# Frame flags
FLAG_OVERFLOW = 1 << 0
FLAG_FD = 1 << 1
FLAG_BRS = 1 << 2
FLAG_ESI = 1 << 3

# can_id bits (SocketCAN)
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF

ECHO_ID_RX = 0xFFFFFFFF  # echo_id of a received frame; TX echoes carry the host's id

# GET_STATE
STATE_ERROR_ACTIVE = 0
STATE_ERROR_WARNING = 1
STATE_ERROR_PASSIVE = 2
STATE_BUS_OFF = 3
STATE_STOPPED = 4
STATE_SLEEPING = 5

HEADER = struct.Struct("<IIBBBB")  # echo_id, can_id, can_dlc, channel, flags, reserved
HEADER_LEN = HEADER.size  # 12

_FD_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


def len_to_dlc(length: int) -> int:
    """Smallest CAN FD DLC code that holds `length` bytes."""
    for code, size in enumerate(_FD_LENGTHS):
        if length <= size:
            return code
    raise ValueError(f"CAN FD payload too long: {length} bytes (max 64)")


def dlc_to_len(dlc: int, fd: bool) -> int:
    """Payload length of a DLC code; classic frames cap at 8."""
    return _FD_LENGTHS[dlc & 0xF] if fd else min(dlc, 8)


def frame_size(flags: int, hw_timestamp: bool) -> int:
    """Bytes one frame takes on the wire: header, 8 or 64 data bytes, then the
    optional 32-bit timestamp right after the data."""
    return HEADER_LEN + (64 if flags & FLAG_FD else 8) + (4 if hw_timestamp else 0)


@dataclass
class Frame:
    echo_id: int
    can_id: int
    dlc: int
    channel: int
    flags: int
    data: bytes  # the payload only (dlc_to_len bytes)
    timestamp_us: int | None = None

    @property
    def is_rx(self) -> bool:
        return self.echo_id == ECHO_ID_RX

    @property
    def is_fd(self) -> bool:
        return bool(self.flags & FLAG_FD)


def pack_frame(echo_id: int, can_id: int, channel: int, data: bytes, fd: bool, brs: bool = False,
               esi: bool = False, dlc: int | None = None) -> bytes:
    """A host-to-device frame (no timestamp: the device adds those on IN only)."""
    flags = 0
    if fd:
        flags = FLAG_FD | (FLAG_BRS if brs else 0) | (FLAG_ESI if esi else 0)
        code = len_to_dlc(len(data)) if dlc is None else dlc
        payload = bytes(data).ljust(64, b"\0")
    else:
        if len(data) > 8:
            raise ValueError("classic CAN frames carry at most 8 bytes")
        code = len(data) if dlc is None else dlc
        payload = bytes(data).ljust(8, b"\0")
    return HEADER.pack(echo_id & 0xFFFFFFFF, can_id & 0xFFFFFFFF, code, channel, flags, 0) + payload


def unpack_frames(buf: bytes, hw_timestamp: bool) -> list[Frame]:
    """All frames in one IN transfer (the probe sends one per transfer; other
    gs_usb devices may pack several). A truncated tail is dropped."""
    frames = []
    off = 0
    while len(buf) - off >= HEADER_LEN + 8:
        echo_id, can_id, dlc, channel, flags, _ = HEADER.unpack_from(buf, off)
        size = frame_size(flags, hw_timestamp)
        if len(buf) - off < size - (4 if hw_timestamp else 0):
            break
        fd = bool(flags & FLAG_FD)
        start = off + HEADER_LEN
        data = bytes(buf[start:start + dlc_to_len(dlc, fd)])
        ts = None
        ts_off = start + (64 if fd else 8)
        if hw_timestamp and len(buf) >= ts_off + 4:
            ts = struct.unpack_from("<I", buf, ts_off)[0]
        frames.append(Frame(echo_id, can_id, dlc, channel, flags, data, ts))
        off += size
    return frames


@dataclass
class BitTimingConst:
    """BT_CONST(_EXT): what the device accepts. Data limits are 0 without FD."""
    feature: int
    fclk_can: int
    tseg1_min: int
    tseg1_max: int
    tseg2_min: int
    tseg2_max: int
    sjw_max: int
    brp_min: int
    brp_max: int
    brp_inc: int
    dtseg1_min: int = 0
    dtseg1_max: int = 0
    dtseg2_min: int = 0
    dtseg2_max: int = 0
    dsjw_max: int = 0
    dbrp_min: int = 0
    dbrp_max: int = 0
    dbrp_inc: int = 0

    @classmethod
    def from_bytes(cls, raw: bytes) -> BitTimingConst:
        n = len(raw) // 4
        return cls(*struct.unpack(f"<{n}I", raw[:n * 4]))


def pack_bittiming(tseg1: int, tseg2: int, sjw: int, brp: int) -> bytes:
    """BITTIMING / DATA_BITTIMING. The device takes tseg1 as prop_seg +
    phase_seg1; one tq of prop_seg is the Linux driver's convention."""
    return struct.pack("<5I", 1, tseg1 - 1, tseg2, sjw, brp)


def pack_mode(mode: int, flags: int) -> bytes:
    return struct.pack("<II", mode, flags)


class TimestampUnwrapper:
    """32-bit microsecond device timestamps to host seconds (wrap every
    ~71.6 min). Anchored to the host clock on the first frame."""

    def __init__(self, now: float):
        self._now = now
        self._anchor: float | None = None
        self._last = 0
        self._high = 0

    def __call__(self, ts_us: int, now: float | None = None) -> float:
        if self._anchor is None:
            self._anchor = (self._now if now is None else now) - ts_us / 1e6
            self._last = ts_us
        if ts_us < self._last and self._last - ts_us > 0x80000000:
            self._high += 1 << 32
        self._last = ts_us
        return self._anchor + (self._high + ts_us) / 1e6
