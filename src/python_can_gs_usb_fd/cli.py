"""gs-usb-fd: detect devices, dump a bus listen-only, or self-test in internal loopback.

    gs-usb-fd detect
    gs-usb-fd listen   [--channel C] [--seconds S] [timing options]
    gs-usb-fd loopback [--channel C] [timing options]

Timing: --bitrate/--data-bitrate (80 % sample point), or the explicit
--nom/--data quadruples (brp tseg1 tseg2 sjw) at the device's CAN clock.
Defaults: 1 Mbit/s nominal, 5 Mbit/s data, as brp/tseg1/tseg2/sjw
5/12/3/3 and 2/5/2/2 at 80 MHz.
"""

from __future__ import annotations

import argparse
import sys
import time

import can

from .bus import INTERFACE

DEFAULT_NOM = (5, 12, 3, 3)
DEFAULT_DATA = (2, 5, 2, 2)
F_CLOCK = 80_000_000


def _timing(a) -> dict:
    if a.bitrate:
        return {"bitrate": a.bitrate, **({"data_bitrate": a.data_bitrate} if a.data_bitrate else {})}
    nom = a.nom or DEFAULT_NOM
    data = a.data or DEFAULT_DATA
    return {"timing": can.BitTimingFd(
        f_clock=a.f_clock,
        nom_brp=nom[0], nom_tseg1=nom[1], nom_tseg2=nom[2], nom_sjw=nom[3],
        data_brp=data[0], data_tseg1=data[1], data_tseg2=data[2], data_sjw=data[3])}


def _fmt(m: can.Message) -> str:
    if m.is_error_frame:
        kind = "ERR    "
    elif m.is_fd:
        kind = "FD BRS " if m.bitrate_switch else "FD     "
    else:
        kind = "CAN    "
    ident = f"{m.arbitration_id:08X}" if m.is_extended_id else f"     {m.arbitration_id:03X}"
    who = "rx  " if m.is_rx else "echo"
    return f"{m.timestamp:17.6f} {who} {kind}{ident} [{m.dlc:2}] {bytes(m.data).hex(' ')}"


def _state(bus) -> str:
    st, rx, tx = bus.error_counters()
    names = {0: "error active", 1: "error warning", 2: "error passive", 3: "bus off", 4: "stopped"}
    return f"{names.get(st, st)}, rx errors {rx}, tx errors {tx}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="gs-usb-fd", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["detect", "listen", "loopback"])
    ap.add_argument("--channel", default="0", help='"SERIAL@N" or N (0-based; probe: 0 = CAN1/J4)')
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--max-print", type=int, default=20)
    ap.add_argument("--bitrate", type=int)
    ap.add_argument("--data-bitrate", type=int)
    ap.add_argument("--nom", type=int, nargs=4, metavar=("BRP", "TSEG1", "TSEG2", "SJW"))
    ap.add_argument("--data", type=int, nargs=4, metavar=("BRP", "TSEG1", "TSEG2", "SJW"))
    ap.add_argument("--f-clock", type=int, default=F_CLOCK)
    a = ap.parse_args(argv)

    if a.cmd == "detect":
        configs = can.detect_available_configs(interfaces=[INTERFACE])
        for c in configs:
            print(c)
        if not configs:
            print("no CAN FD gs_usb device found (or all in use)")
        return 0 if configs else 1

    if a.cmd == "listen":
        with can.Bus(interface=INTERFACE, channel=a.channel, listen_only=True, **_timing(a)) as bus:
            print(bus.channel_info)
            n = fd = brs = 0
            ids: dict[int, int] = {}
            t_end = time.monotonic() + a.seconds
            while time.monotonic() < t_end:
                m = bus.recv(0.2)
                if m is None:
                    continue
                n += 1
                fd += m.is_fd
                brs += m.bitrate_switch
                ids[m.arbitration_id] = ids.get(m.arbitration_id, 0) + 1
                if n <= a.max_print:
                    print(_fmt(m))
            print(f"{n} frames in {a.seconds:g} s ({fd} FD, {brs} BRS), {len(ids)} IDs; {_state(bus)}")
        return 0

    sent = [
        can.Message(arbitration_id=0x123, data=b"\xde\xad\xbe\xef", is_extended_id=False),
        can.Message(arbitration_id=0x1ABCDE0, data=bytes(range(64)), is_extended_id=True,
                    is_fd=True, bitrate_switch=True),
        can.Message(arbitration_id=0x7FF, data=bytes(range(12)), is_extended_id=False, is_fd=True),
    ]
    with can.Bus(interface=INTERFACE, channel=a.channel, loopback=True, receive_own_messages=True,
                 **_timing(a)) as bus:
        print(bus.channel_info)
        got = []
        for s in sent:
            bus.send(s, timeout=1.0)
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            m = bus.recv(0.1)
            if m is not None:
                print(_fmt(m))
                got.append(m)
        state = _state(bus)
    rx = [g for g in got if g.is_rx]
    ok = all(any(g.arbitration_id == s.arbitration_id and bytes(g.data) == bytes(s.data)
                 and g.is_fd == s.is_fd and g.bitrate_switch == s.bitrate_switch for g in rx) for s in sent)
    print(f"{'PASS' if ok else 'FAIL'}: sent {len(sent)}, received {len(rx)} (+{len(got) - len(rx)} echoes); {state}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
