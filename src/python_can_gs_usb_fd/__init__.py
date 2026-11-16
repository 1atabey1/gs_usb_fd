"""python-can interface "gs_usb_fd": CAN FD over gs_usb devices.

Installing this package registers the interface with python-can (entry point
group ``can.interface``), so ``can.Bus(interface="gs_usb_fd", ...)`` works
without importing anything from here.
"""

from .bus import INTERFACE, GsUsbFdBus, parse_channel

__all__ = ["INTERFACE", "GsUsbFdBus", "parse_channel"]
__version__ = "0.1.0"
