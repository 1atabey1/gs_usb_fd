# python-can-gs-usb-fd

A [python-can](https://python-can.readthedocs.io/) interface for **CAN FD over gs_usb** devices: the tricore-debug-machine probe (USB 1d50:606f, interface 0) and other gs_usb devices that advertise CAN FD.

Installing the package registers the interface `gs_usb_fd` with python-can (entry point group `can.interface`). Code only needs `import can`, and every python-can tool (`can.logger`, `can.viewer`, `can.player`, ...) can use it.

> **Why not python-can's `gs_usb` interface?** It does classic CAN only: no FD mode, no data-phase bit timing, and a fixed 87.5 % sample point. On a CAN FD network, a controller in classic mode error-frames every FD frame. The probe's firmware runs FDCAN with protocol exception handling off, so it would do exactly that.

## Installation

In the repo's uv environment (`sw/`) it is already an editable workspace member:

```bash
uv sync
```

Elsewhere:

```bash
pip install ./tools/python-can-gs-usb-fd
```

On Windows the device needs WinUSB on its gs_usb interface. The probe gets WinUSB automatically through MS OS 2.0 descriptors, so no driver install is needed. libusb-1.0 comes from the `libusb-package` dependency. Do **not** install libusb-win32: pyusb would fall back to its `libusb0.dll`, which cannot see WinUSB devices, and its filter driver can hang Windows device management.

## Configuration

The interface is always `"gs_usb_fd"`. The `channel` is `"SERIAL@N"`, or just `N` for the first device found. Channels are **0-based**, as the device numbers them. On the probe, 0 is CAN1 (J4) and 1 is CAN2 (J5).

```python
configs = can.detect_available_configs(interfaces=["gs_usb_fd"])
# [{'interface': 'gs_usb_fd', 'channel': '290023001551333034363238@0'},
#  {'interface': 'gs_usb_fd', 'channel': '290023001551333034363238@1'}]
```

Detection opens each device briefly, so devices that another process has open are skipped.

## Opening a bus

```python
import can

with can.Bus(interface="gs_usb_fd", channel=0, bitrate=1_000_000, data_bitrate=5_000_000) as bus:
    bus.send(can.Message(arbitration_id=0x123, data=bytes(64), is_fd=True, bitrate_switch=True))
    print(bus.recv(timeout=1.0))
```

### Bit timing

`bitrate` (plus `data_bitrate` for CAN FD) uses an 80 % sample point; `sample_point` and `data_sample_point` change it. For full control, pass `timing`: a `can.BitTimingFd` for CAN FD or a `can.BitTiming` for classic CAN. The probe's CAN clock is **80 MHz**. A timing made for another clock is rescaled if it divides evenly, otherwise it is refused.

```python
timing = can.BitTimingFd(f_clock=80_000_000,
                         nom_brp=5, nom_tseg1=12, nom_tseg2=3, nom_sjw=3,   # 1 Mbit/s, 81.25 %
                         data_brp=2, data_tseg1=5, data_tseg2=2, data_sjw=2)  # 5 Mbit/s, 75 %
with can.Bus(interface="gs_usb_fd", channel="290023001551333034363238@0", timing=timing) as bus:
    ...
```

Without `data_bitrate` and with a `can.BitTiming`, the channel runs classic CAN, and sending an FD frame raises `CanOperationError`.

### Modes

| Argument | Effect |
|---|---|
| `listen_only=True` | Never drives the bus: no ACK, no error frames, no TX (FDCAN bus monitoring). Safe on a live network. |
| `loopback=True` | Internal loopback: TX frames come back as RX and the bus is not touched. |
| `one_shot=True` | No automatic retransmission. |
| `error_frames=True` | Bus-error reporting on; error frames arrive with `is_error_frame`, with `arbitration_id`/`data` in the SocketCAN error-frame format (`linux/can/error.h`). Off by default, and error frames are dropped. |
| `receive_own_messages=True` | The device's TX confirmations are delivered too (`is_rx` False). |
| `hw_timestamps=False` | Host receive time instead of the device's microsecond timestamps (default: device time, anchored to the host clock and unwrapped). |

### Several channels

Both channels of a device can be open at once, in one process: they share one USB connection and one reader thread that routes frames by channel.

```python
with (can.Bus(interface="gs_usb_fd", channel=0, timing=timing) as can1,
      can.Bus(interface="gs_usb_fd", channel=1, timing=timing) as can2):
    ...
```

### Filters, state

`can_filters` are applied in software (the device has no hardware filters). `bus.state` reads the controller state: `ACTIVE` (error-active or warning), `PASSIVE` or `ERROR` (bus-off). `bus.error_counters()` returns `(state, rx_errors, tx_errors)`.

## Command line

```bash
gs-usb-fd detect
gs-usb-fd listen   --channel 0 --seconds 3     # listen-only dump, never drives the bus
gs-usb-fd loopback --channel 0                 # internal loopback self-test
```

Both default to 1 Mbit/s / 5 Mbit/s (brp/tseg1/tseg2/sjw 5/12/3/3 and 2/5/2/2 at 80 MHz). Use `--bitrate`/`--data-bitrate` or `--nom`/`--data` to change it.

## Tests

```bash
uv run pytest tools/python-can-gs-usb-fd                      # a fake device, no hardware
GS_USB_FD_HW=1 uv run pytest tools/python-can-gs-usb-fd -m hardware   # on a probe: loopback and listen-only only
```

## Protocol

The Linux gs_usb host format with CAN FD (`drivers/net/can/usb/gs_usb.c`, candleLight_fw). The probe's side is `Appli/usb/gs_usb.c`: BT_CONST_EXT, DATA_BITTIMING, frames of a 12-byte header plus 8 or 64 data bytes plus a 32-bit timestamp, and one frame per USB transfer.
