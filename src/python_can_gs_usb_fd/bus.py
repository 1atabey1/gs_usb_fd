"""python-can bus for CAN FD over gs_usb (interface name "gs_usb_fd")."""

from __future__ import annotations

import itertools
import queue
import struct
import time
import warnings
from typing import Any

import can
import usb.core

from . import device as _device
from . import protocol as p
from .device import GsUsbDevice, _serial

INTERFACE = "gs_usb_fd"
DEFAULT_SAMPLE_POINT = 80.0


def parse_channel(channel: Any) -> tuple[str | None, int]:
    """0, "0" -> (None, 0): channel 0 of the first device found;
    "SERIAL@1" -> ("SERIAL", 1). Channels are 0-based, as the device numbers
    them (on the tricore-debug-machine probe 0 is CAN1 / J4, 1 is CAN2 / J5)."""
    if channel is None:
        return None, 0
    if isinstance(channel, int):
        return None, channel
    text = str(channel).strip()
    serial, sep, index = text.rpartition("@")
    if not sep:
        return None, int(text)
    return (serial or None), int(index)


class GsUsbFdBus(can.BusABC):
    """CAN FD (and classic CAN) over a gs_usb device.

    python-can's built-in ``gs_usb`` interface is classic CAN only. On a CAN
    FD network a classic-mode controller error-frames every FD frame, so use
    this interface there.

    Bit timing, in order of precedence:

    * ``timing``: a :class:`can.BitTimingFd` (CAN FD) or :class:`can.BitTiming`
      (classic CAN). Its ``f_clock`` must be the device's CAN clock (80 MHz on
      the tricore-debug-machine probe), or be adjustable to it.
    * ``bitrate`` (and ``data_bitrate`` for CAN FD), with a fixed sample point
      of 80 % (``sample_point`` / ``data_sample_point`` change it).

    :param channel: ``"SERIAL@N"`` or ``N`` (the first device found), N 0-based.
    :param listen_only: never drive the bus: no ACK, no error frames, no TX.
    :param loopback: internal loopback: TX frames come back as RX, the bus is
        not touched (sets the device's listen-only and loopback flags, which the
        tricore-debug-machine firmware maps to FDCAN internal loopback).
    :param one_shot: no automatic retransmission.
    :param error_frames: enable bus-error reporting and deliver error frames
        (``is_error_frame``; ``arbitration_id`` and ``data`` in the SocketCAN
        error-frame format). Off by default: error frames are dropped.
    :param receive_own_messages: deliver the device's TX echoes (``is_rx`` False).
    :param hw_timestamps: use the device's microsecond timestamps (default),
        anchored to the host clock; else the host receive time.
    """

    def __init__(
        self,
        channel: Any = 0,
        can_filters: can.typechecking.CanFilters | None = None,
        bitrate: int | None = None,
        data_bitrate: int | None = None,
        timing: can.BitTiming | can.BitTimingFd | None = None,
        sample_point: float = DEFAULT_SAMPLE_POINT,
        data_sample_point: float = DEFAULT_SAMPLE_POINT,
        listen_only: bool = False,
        loopback: bool = False,
        one_shot: bool = False,
        error_frames: bool = False,
        receive_own_messages: bool = False,
        hw_timestamps: bool = True,
        **kwargs: Any,
    ) -> None:
        self._dev: GsUsbDevice | None = None
        self._queue: queue.Queue | None = None
        self._started = False
        self._channel_open = False  # this bus registered its channel with the device
        serial, self._ch = parse_channel(channel)
        try:
            self._dev = GsUsbDevice.acquire(serial)
        except (LookupError, usb.core.USBError, usb.core.NoBackendError) as e:
            raise can.CanInitializationError(f"{INTERFACE}: {e}") from e
        try:
            self._setup(timing, bitrate, data_bitrate, sample_point, data_sample_point,
                        listen_only, loopback, one_shot, error_frames, hw_timestamps)
        except Exception:
            self._release()
            raise
        self._error_frames = error_frames
        self._receive_own = receive_own_messages
        self._echo_ids = itertools.count(1)
        self.channel_info = (
            f"{INTERFACE} {self._dev.serial}@{self._ch}: "
            + (f"{self.timing.nom_bitrate / 1e6:g} / {self.timing.data_bitrate / 1e6:g} Mbit/s FD"
               if self._fd else f"{self.timing.bitrate / 1e6:g} Mbit/s classic")
            + (", listen-only" if listen_only and not loopback else "")
            + (", internal loopback" if loopback else ""))
        super().__init__(channel=channel, can_filters=can_filters, **kwargs)

    # ---- setup
    def _setup(self, timing, bitrate, data_bitrate, sample_point, data_sample_point,
               listen_only, loopback, one_shot, error_frames, hw_timestamps) -> None:
        dev = self._dev
        if not 0 <= self._ch < dev.channel_count:
            raise can.CanInitializationError(
                f"{INTERFACE}: channel {self._ch} out of range, {dev.serial} has {dev.channel_count}")
        btc = dev.bt_const
        fclk = btc.fclk_can

        if isinstance(timing, can.BitTimingFd):
            self._fd = True
            self.timing = can.util.check_or_adjust_timing_clock(timing, valid_clocks=[fclk])
        elif isinstance(timing, can.BitTiming):
            self._fd = False
            self.timing = can.util.check_or_adjust_timing_clock(timing, valid_clocks=[fclk])
        elif bitrate and data_bitrate:
            self._fd = True
            self.timing = can.BitTimingFd.from_sample_point(
                f_clock=fclk, nom_bitrate=bitrate, nom_sample_point=sample_point,
                data_bitrate=data_bitrate, data_sample_point=data_sample_point)
        elif bitrate:
            self._fd = False
            self.timing = can.BitTiming.from_sample_point(f_clock=fclk, bitrate=bitrate,
                                                          sample_point=sample_point)
        else:
            raise can.CanInitializationError(f"{INTERFACE}: give bitrate (and data_bitrate) or timing")

        if self._fd and not btc.feature & p.FEATURE_FD:
            raise can.CanInitializationError(f"{INTERFACE}: {dev.serial} does not support CAN FD")
        want = (p.FEATURE_LISTEN_ONLY if listen_only or loopback else 0) \
            | (p.FEATURE_LOOP_BACK if loopback else 0) \
            | (p.FEATURE_ONE_SHOT if one_shot else 0) \
            | (p.FEATURE_BERR_REPORTING if error_frames else 0) \
            | (p.FEATURE_HW_TIMESTAMP if hw_timestamps else 0)
        missing = want & ~btc.feature
        if missing & ~p.FEATURE_HW_TIMESTAMP:
            raise can.CanInitializationError(f"{INTERFACE}: {dev.serial} lacks mode flags 0x{missing:x}")
        hw_timestamps = hw_timestamps and bool(btc.feature & p.FEATURE_HW_TIMESTAMP)
        flags = (want & btc.feature) | (p.FEATURE_FD if self._fd else 0)

        # Claim the channel first: a channel another bus has open must not be
        # reset or retimed under it.
        try:
            self._queue = dev.open_channel(self._ch, hw_timestamps)
        except RuntimeError as e:
            raise can.CanInitializationError(f"{INTERFACE}: {e}") from e
        self._channel_open = True
        try:
            dev.ctrl_out(p.BREQ_MODE, p.pack_mode(p.MODE_RESET, 0), value=self._ch)
            if self._fd:
                t = self.timing
                dev.ctrl_out(p.BREQ_BITTIMING, p.pack_bittiming(t.nom_tseg1, t.nom_tseg2, t.nom_sjw, t.nom_brp),
                             value=self._ch)
                dev.ctrl_out(p.BREQ_DATA_BITTIMING,
                             p.pack_bittiming(t.data_tseg1, t.data_tseg2, t.data_sjw, t.data_brp), value=self._ch)
            else:
                t = self.timing
                dev.ctrl_out(p.BREQ_BITTIMING, p.pack_bittiming(t.tseg1, t.tseg2, t.sjw, t.brp), value=self._ch)
            dev.ctrl_out(p.BREQ_MODE, p.pack_mode(p.MODE_START, flags), value=self._ch)
            self._started = True
        except usb.core.USBError as e:
            raise can.CanInitializationError(f"{INTERFACE}: {e}") from e
        self._can_protocol = can.CanProtocol.CAN_FD if self._fd else can.CanProtocol.CAN_20

    # ---- transmit / receive
    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        if msg.is_fd and not self._fd:
            raise can.CanOperationError(f"{INTERFACE}: CAN FD frame on a classic CAN bus")
        can_id = msg.arbitration_id | (p.CAN_EFF_FLAG if msg.is_extended_id else 0)
        if msg.is_remote_frame:
            can_id |= p.CAN_RTR_FLAG
        data = bytes(msg.data)
        frame = p.pack_frame(next(self._echo_ids) & 0x7FFFFFFF, can_id, self._ch, data, fd=msg.is_fd,
                             brs=msg.bitrate_switch, esi=msg.error_state_indicator,
                             dlc=None if msg.is_fd or not msg.is_remote_frame else msg.dlc)
        try:
            self._dev.write(frame, int(1000 * (1.0 if timeout is None else max(timeout, 0.001))))
        except usb.core.USBTimeoutError as e:
            raise can.CanOperationError(f"{INTERFACE}: the device did not take the frame in time") from e
        except usb.core.USBError as e:
            raise can.CanOperationError(f"{INTERFACE}: {e}") from e

    def _recv_internal(self, timeout: float | None) -> tuple[can.Message | None, bool]:
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            err = self._dev.reader_error()
            if err is not None and self._queue.empty():
                raise can.CanOperationError(f"{INTERFACE}: USB read failed: {err}")
            wait = 0.1 if deadline is None else max(0.0, min(0.1, deadline - time.monotonic()))
            try:
                ts, f = self._queue.get(timeout=wait)
            except queue.Empty:
                if deadline is not None and time.monotonic() >= deadline:
                    return None, False
                continue
            if f.can_id & p.CAN_ERR_FLAG and not self._error_frames:
                continue
            if not f.is_rx and not self._receive_own:
                continue
            if f.flags & p.FLAG_OVERFLOW:
                warnings.warn(f"{INTERFACE}: the device dropped frames (overflow)", RuntimeWarning, stacklevel=2)
            return self._to_message(ts, f), False

    def _to_message(self, ts: float, f: p.Frame) -> can.Message:
        err = bool(f.can_id & p.CAN_ERR_FLAG)
        ext = bool(f.can_id & p.CAN_EFF_FLAG)
        if err:
            arb = f.can_id & p.CAN_EFF_MASK
        else:
            arb = f.can_id & (p.CAN_EFF_MASK if ext else p.CAN_SFF_MASK)
        return can.Message(
            timestamp=ts,
            arbitration_id=arb,
            is_extended_id=ext and not err,
            is_remote_frame=bool(f.can_id & p.CAN_RTR_FLAG),
            is_error_frame=err,
            is_fd=f.is_fd,
            bitrate_switch=bool(f.flags & p.FLAG_BRS),
            error_state_indicator=bool(f.flags & p.FLAG_ESI),
            is_rx=f.is_rx,
            channel=self._ch,
            dlc=len(f.data) if f.is_fd else min(f.dlc, 8),
            data=f.data,
        )

    # ---- state
    @property
    def state(self) -> can.BusState:
        if not self._started or self._dev is None:
            raise can.CanOperationError(f"{INTERFACE}: the bus is closed")
        if not self._dev.bt_const.feature & p.FEATURE_GET_STATE:
            return can.BusState.ACTIVE
        st, _rx, _tx = self.error_counters()
        if st in (p.STATE_ERROR_ACTIVE, p.STATE_ERROR_WARNING):
            return can.BusState.ACTIVE
        if st == p.STATE_ERROR_PASSIVE:
            return can.BusState.PASSIVE
        if st == p.STATE_BUS_OFF:
            return can.BusState.ERROR
        raise can.CanOperationError(f"{INTERFACE}: channel {self._ch} is stopped")

    @state.setter
    def state(self, new_state: can.BusState) -> None:
        raise NotImplementedError(f"{INTERFACE}: set listen_only when opening the bus")

    def error_counters(self) -> tuple[int, int, int]:
        """(state, rx error count, tx error count) from GET_STATE."""
        try:
            return struct.unpack("<3I", self._dev.ctrl_in(p.BREQ_GET_STATE, 12, value=self._ch))
        except usb.core.USBError as e:
            raise can.CanOperationError(f"{INTERFACE}: {e}") from e

    # ---- shutdown
    def _release(self) -> None:
        if self._dev is None:
            return
        if self._started:
            try:
                self._dev.ctrl_out(p.BREQ_MODE, p.pack_mode(p.MODE_RESET, 0), value=self._ch)
            except usb.core.USBError:
                pass
            self._started = False
        if self._channel_open:
            self._dev.close_channel(self._ch)
            self._channel_open = False
        self._dev.release()
        self._dev = None

    def shutdown(self) -> None:
        self._release()
        super().shutdown()

    # ---- discovery
    @staticmethod
    def _detect_available_configs() -> list[can.typechecking.AutoDetectedConfig]:
        configs: list[can.typechecking.AutoDetectedConfig] = []
        try:
            usb_devs = _device.find_usb_devices()
        except usb.core.NoBackendError:
            return configs
        for u in usb_devs:
            serial = _serial(u)
            try:
                dev = GsUsbDevice.acquire(serial, usb_dev=u)
            except (LookupError, usb.core.USBError):
                continue  # in use by another process, or not a gs_usb device
            try:
                if not dev.bt_const.feature & p.FEATURE_FD:
                    continue
                for ch in range(dev.channel_count):
                    configs.append({"interface": INTERFACE, "channel": f"{serial}@{ch}"})
            finally:
                dev.release()
        return configs
