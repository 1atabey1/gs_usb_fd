import struct
from importlib.metadata import entry_points

import can
import pytest

from python_can_gs_usb_fd import GsUsbFdBus, parse_channel
from python_can_gs_usb_fd import protocol as p

PCAN = dict(f_clock=80_000_000, nom_brp=5, nom_tseg1=12, nom_tseg2=3, nom_sjw=3,
            data_brp=2, data_tseg1=5, data_tseg2=2, data_sjw=2)


def test_entry_point_registers_interface():
    eps = {ep.name: ep.value for ep in entry_points(group="can.interface")}
    assert eps.get("gs_usb_fd") == "python_can_gs_usb_fd.bus:GsUsbFdBus"
    assert "gs_usb_fd" in can.interfaces.VALID_INTERFACES


def test_parse_channel():
    assert parse_channel(0) == (None, 0)
    assert parse_channel("1") == (None, 1)
    assert parse_channel("290023001551333034363238@1") == ("290023001551333034363238", 1)
    assert parse_channel(None) == (None, 0)


def test_open_with_explicit_fd_timing(fake):
    with can.Bus(interface="gs_usb_fd", channel=0, timing=can.BitTimingFd(**PCAN)) as bus:
        assert isinstance(bus, GsUsbFdBus)
        assert bus.protocol == can.CanProtocol.CAN_FD
        assert "1 / 5 Mbit/s FD" in bus.channel_info
    # tseg1 = prop_seg 1 + phase_seg1
    assert fake.bittiming[0] == (1, 11, 3, 3, 5)
    assert fake.data_bittiming[0] == (1, 4, 2, 2, 2)
    starts = [struct.unpack("<II", d) for r, v, d in fake.requests if r == p.BREQ_MODE and v == 0]
    assert starts[-2][0] == p.MODE_START and starts[-2][1] & p.FEATURE_FD
    assert starts[-1] == (p.MODE_RESET, 0)  # shutdown resets the channel


def test_open_with_bitrates_and_classic(fake):
    with can.Bus(interface="gs_usb_fd", channel=1, bitrate=500_000, data_bitrate=2_000_000) as bus:
        assert bus.protocol == can.CanProtocol.CAN_FD
    with can.Bus(interface="gs_usb_fd", channel=1, bitrate=500_000) as bus:
        assert bus.protocol == can.CanProtocol.CAN_20
        with pytest.raises(can.CanOperationError):
            bus.send(can.Message(arbitration_id=1, data=b"\x00", is_fd=True))
        assert not fake.mode[1][1] & p.FEATURE_FD


def test_loopback_send_receive_and_own_messages(fake):
    with can.Bus(interface="gs_usb_fd", channel=0, timing=can.BitTimingFd(**PCAN), loopback=True) as bus:
        assert fake.mode[0][1] & p.FEATURE_LOOP_BACK and fake.mode[0][1] & p.FEATURE_LISTEN_ONLY
        bus.send(can.Message(arbitration_id=0x1ABCDE0, is_extended_id=True, data=bytes(range(64)),
                             is_fd=True, bitrate_switch=True))
        m = bus.recv(1.0)
        assert m.is_rx and m.is_fd and m.bitrate_switch and m.is_extended_id
        assert m.arbitration_id == 0x1ABCDE0 and bytes(m.data) == bytes(range(64))
        assert bus.recv(0.2) is None  # the echo is not delivered by default
    with can.Bus(interface="gs_usb_fd", channel=0, timing=can.BitTimingFd(**PCAN), loopback=True,
                 receive_own_messages=True) as bus:
        bus.send(can.Message(arbitration_id=0x123, data=b"\x01\x02", is_extended_id=False))
        got = [bus.recv(1.0), bus.recv(1.0)]
        assert sorted(m.is_rx for m in got) == [False, True]


def test_two_channels_share_one_device(fake):
    t = can.BitTimingFd(**PCAN)
    with can.Bus(interface="gs_usb_fd", channel=0, timing=t) as can1, \
            can.Bus(interface="gs_usb_fd", channel=1, timing=t) as can2:
        fake.inject(1, 0x222, b"\x02")
        fake.inject(0, 0x111, b"\x01" * 12, fd=True, brs=True)
        m1, m2 = can1.recv(1.0), can2.recv(1.0)
        assert (m1.arbitration_id, m1.channel, m1.dlc) == (0x111, 0, 12)
        assert (m2.arbitration_id, m2.channel) == (0x222, 1)
        resets = len([r for r in fake.requests if r[0] == p.BREQ_MODE])
        with pytest.raises(can.CanInitializationError):
            can.Bus(interface="gs_usb_fd", channel=1, timing=t)  # already open
        # the refused open neither reset the channel nor unregistered it
        assert len([r for r in fake.requests if r[0] == p.BREQ_MODE]) == resets
        fake.inject(1, 0x333, b"\x03")
        m = can2.recv(1.0)
        assert m is not None and m.arbitration_id == 0x333


def test_error_frames_are_opt_in(fake):
    t = can.BitTimingFd(**PCAN)
    err_id = p.CAN_ERR_FLAG | 0x00000004
    with can.Bus(interface="gs_usb_fd", channel=0, timing=t) as bus:
        fake.inject(0, err_id, bytes(8))
        assert bus.recv(0.3) is None
    with can.Bus(interface="gs_usb_fd", channel=0, timing=t, error_frames=True) as bus:
        assert fake.mode[0][1] & p.FEATURE_BERR_REPORTING
        fake.inject(0, err_id, bytes([0, 0x10, 0, 0, 0, 0, 0, 0]))
        m = bus.recv(1.0)
        assert m.is_error_frame and m.arbitration_id == 0x4 and m.data[1] == 0x10


def test_software_filters(fake):
    with can.Bus(interface="gs_usb_fd", channel=0, timing=can.BitTimingFd(**PCAN),
                 can_filters=[{"can_id": 0x200, "can_mask": 0x7FF, "extended": False}]) as bus:
        fake.inject(0, 0x100, b"\x01")
        fake.inject(0, 0x200, b"\x02")
        m = bus.recv(1.0)
        assert m.arbitration_id == 0x200


def test_state(fake):
    with can.Bus(interface="gs_usb_fd", channel=0, timing=can.BitTimingFd(**PCAN)) as bus:
        assert bus.state == can.BusState.ACTIVE
        fake.state = (p.STATE_ERROR_PASSIVE, 130, 0)
        assert bus.state == can.BusState.PASSIVE
        fake.state = (p.STATE_BUS_OFF, 0, 256)
        assert bus.state == can.BusState.ERROR


def test_detect_available_configs(fake):
    configs = can.detect_available_configs(interfaces=["gs_usb_fd"])
    assert configs == [{"interface": "gs_usb_fd", "channel": "FAKE0001@0"},
                       {"interface": "gs_usb_fd", "channel": "FAKE0001@1"}]


def test_bad_channel_and_missing_timing(fake):
    with pytest.raises(can.CanInitializationError):
        can.Bus(interface="gs_usb_fd", channel=5, bitrate=500_000)
    with pytest.raises(can.CanInitializationError):
        can.Bus(interface="gs_usb_fd", channel=0)
    with pytest.raises(can.CanInitializationError):
        can.Bus(interface="gs_usb_fd", channel="OTHER@0", bitrate=500_000)


def test_wrong_clock_timing_is_adjusted_or_refused(fake):
    t40 = can.BitTimingFd(f_clock=40_000_000, nom_brp=5, nom_tseg1=12, nom_tseg2=3, nom_sjw=3,
                          data_brp=1, data_tseg1=5, data_tseg2=2, data_sjw=2)
    with pytest.warns(UserWarning, match="Adjusted f_clock"):  # 40 MHz -> 80 MHz, brp doubled
        bus = can.Bus(interface="gs_usb_fd", channel=0, timing=t40)
    with bus:
        assert bus.timing.f_clock == 80_000_000 and bus.timing.nom_bitrate == 500_000
