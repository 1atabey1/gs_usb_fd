"""On a real probe (GS_USB_FD_HW=1). Internal loopback and listen-only only:
neither ever drives the bus."""

import can
import pytest

PCAN = dict(f_clock=80_000_000, nom_brp=5, nom_tseg1=12, nom_tseg2=3, nom_sjw=3,
            data_brp=2, data_tseg1=5, data_tseg2=2, data_sjw=2)

pytestmark = pytest.mark.hardware


def test_detect_finds_the_probe():
    configs = can.detect_available_configs(interfaces=["gs_usb_fd"])
    assert len(configs) >= 2


@pytest.mark.parametrize("channel", [0, 1])
def test_internal_loopback(channel):
    with can.Bus(interface="gs_usb_fd", channel=channel, timing=can.BitTimingFd(**PCAN), loopback=True) as bus:
        sent = can.Message(arbitration_id=0x18DAF110, is_extended_id=True, data=bytes(range(48)),
                           is_fd=True, bitrate_switch=True)
        bus.send(sent, timeout=1.0)
        m = bus.recv(1.0)
        assert m is not None and m.is_rx
        assert (m.arbitration_id, bytes(m.data), m.is_fd, m.bitrate_switch) == \
            (sent.arbitration_id, bytes(sent.data), True, True)
        assert bus.state == can.BusState.ACTIVE


def test_listen_only_opens():
    with can.Bus(interface="gs_usb_fd", channel=0, timing=can.BitTimingFd(**PCAN), listen_only=True) as bus:
        bus.recv(0.5)  # traffic or not, the bus must open and read without error
        assert bus.state in (can.BusState.ACTIVE, can.BusState.PASSIVE)
