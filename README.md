# python-can-gs-usb-fd

A [python-can](https://python-can.readthedocs.io/) interface for **CAN FD over gs_usb** devices: the tricore-debug-machine probe (USB 1d50:606f) and other gs_usb devices that advertise CAN FD.

Installing the package registers the interface `gs_usb_fd` with python-can (entry point group `can.interface`). Code only needs `import can`, and every python-can tool (`can.logger`, `can.viewer`, `can.player`, ...) can use it.

> **Why not python-can's `gs_usb` interface?** It does classic CAN only: no FD mode, no data-phase bit timing, and a fixed 87.5 % sample point. On a CAN FD network, a controller in classic mode error-frames every FD frame. The probe's firmware runs FDCAN with protocol exception handling off, so it would do exactly that.

## Installation

```bash
pip install git+https://github.com/1atabey1/gs_usb_fd
# or, in a uv project:
uv add git+https://github.com/1atabey1/gs_usb_fd
```

### Windows

The device needs WinUSB on its gs_usb interface. The probe gets WinUSB automatically through MS OS 2.0 descriptors, so no driver install is needed.

The interface opens **only the gs_usb interface** it uses, directly through `winusb.dll` (found by candleLight's DeviceInterfaceGUID `{c15b4308-04d3-11e6-b3ea-6057189e6443}`). libusb would open the whole composite device instead, and WinUSB gives out one handle per interface, so a CAN program would lock every other program out of the probe's GDB, DAP and trace interfaces. Devices without that GUID (other drivers) fall back to libusb. `GS_USB_FD_BACKEND=libusb` forces libusb.

libusb-1.0 comes from the `libusb-package` dependency. Do **not** install libusb-win32: pyusb would fall back to its `libusb0.dll`, which cannot see WinUSB devices, and its filter driver can hang Windows device management.

### Linux

pyusb and libusb-1.0. The kernel's `gs_usb` driver binds a probe's interface 0 (can0/can1); opening a bus here detaches it from the interface (until the probe is plugged in again). Your user needs access to the device node (the probe's udev rule).

## Configuration

The interface is always `"gs_usb_fd"`. The `channel` is `"SERIAL@N"`, or just `N` for the first device found. Channels are **0-based**, as the device numbers them. On the probe, 0 is CAN1 (J4) and 1 is CAN2 (J5).

A device can have several gs_usb interfaces. Channels are then counted across them in interface order, so channel numbers do not depend on the layout. The probe has two USB layouts (`tdm_dap.py usb-mode`):

| Layout | gs_usb interfaces | Channel 0 | Channel 1 |
|---|---|---|---|
| `combined` (default) | 0 (2 channels) | interface 0, channel 0 | interface 0, channel 1 |
| `split` | 0 and 4 (1 channel each) | interface 0 | interface 4 |

In `split` two programs can each have one channel. In `combined` the program that opens the first channel holds both. An interface another program holds cannot be asked for its channel count, so it counts as one channel.

```python
configs = can.detect_available_configs(interfaces=["gs_usb_fd"])
# [{'interface': 'gs_usb_fd', 'channel': '290023001551333034363238@0'},
#  {'interface': 'gs_usb_fd', 'channel': '290023001551333034363238@1'}]
```

Detection opens each gs_usb interface briefly, so channels on an interface that another process has open are skipped.

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

Both channels of a device can be open at once, in one process. Channels on the same gs_usb interface share one USB connection and one reader thread that routes frames by channel. With the probe in its `split` layout, each channel can also go to a different process.

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
uv run --extra test pytest                                # a fake device, no hardware
GS_USB_FD_HW=1 uv run --extra test pytest -m hardware     # on a probe: loopback and listen-only only
```

## Protocol

The Linux gs_usb host format with CAN FD (`drivers/net/can/usb/gs_usb.c`, candleLight_fw). The probe's side is `sw/Appli/usb/gs_usb.c` in the tricore-debug-machine firmware: BT_CONST_EXT, DATA_BITTIMING, frames of a 12-byte header plus 8 or 64 data bytes plus a 32-bit timestamp, and one frame per USB transfer.
