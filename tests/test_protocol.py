import pytest

from python_can_gs_usb_fd import protocol as p


def test_dlc_mapping():
    assert [p.len_to_dlc(n) for n in (0, 8, 9, 12, 13, 33, 64)] == [0, 8, 9, 9, 10, 14, 15]
    assert p.dlc_to_len(15, fd=True) == 64
    assert p.dlc_to_len(15, fd=False) == 8
    with pytest.raises(ValueError):
        p.len_to_dlc(65)


def test_pack_classic_and_fd():
    classic = p.pack_frame(7, 0x123, 1, b"\x01\x02", fd=False)
    assert len(classic) == 12 + 8
    assert p.HEADER.unpack_from(classic) == (7, 0x123, 2, 1, 0, 0)
    fd = p.pack_frame(8, 0x1ABCDE0 | p.CAN_EFF_FLAG, 0, bytes(range(20)), fd=True, brs=True)
    assert len(fd) == 12 + 64
    echo, can_id, dlc, ch, flags, _ = p.HEADER.unpack_from(fd)
    assert (can_id, dlc, ch, flags) == (0x1ABCDE0 | p.CAN_EFF_FLAG, 11, 0, p.FLAG_FD | p.FLAG_BRS)
    with pytest.raises(ValueError):
        p.pack_frame(1, 0x1, 0, bytes(9), fd=False)


def test_unpack_several_frames_with_timestamps():
    a = p.pack_frame(p.ECHO_ID_RX, 0x100, 0, b"\xaa" * 16, fd=True, brs=True) + (1234).to_bytes(4, "little")
    b = p.pack_frame(p.ECHO_ID_RX, 0x7FF, 1, b"\x01", fd=False) + (1300).to_bytes(4, "little")
    frames = p.unpack_frames(a + b, hw_timestamp=True)
    assert [(f.can_id, f.channel, f.data, f.timestamp_us, f.is_fd) for f in frames] == [
        (0x100, 0, b"\xaa" * 16, 1234, True),
        (0x7FF, 1, b"\x01", 1300, False),
    ]
    assert all(f.is_rx for f in frames)


def test_unpack_without_timestamps_and_truncated_tail():
    a = p.pack_frame(p.ECHO_ID_RX, 0x10, 0, b"\x05", fd=False)
    frames = p.unpack_frames(a + a[:10], hw_timestamp=False)
    assert len(frames) == 1 and frames[0].timestamp_us is None


def test_bittiming_payload_splits_tseg1():
    assert p.pack_bittiming(12, 3, 3, 5) == (1).to_bytes(4, "little") + (11).to_bytes(4, "little") + \
        (3).to_bytes(4, "little") + (3).to_bytes(4, "little") + (5).to_bytes(4, "little")


def test_timestamp_unwrap():
    clock = p.TimestampUnwrapper(now=100.0)
    t0 = clock(0xFFFFFF00)
    assert t0 == pytest.approx(100.0)
    t1 = clock(0x00000100)  # wrapped: 0x200 us later
    assert t1 - t0 == pytest.approx(0x200 / 1e6)
