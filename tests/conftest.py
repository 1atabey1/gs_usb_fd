import os

import pytest
import usb.util

from python_can_gs_usb_fd import device

from fake_device import FakeGsUsb


def _install(monkeypatch, devs):
    # libusb path only: on Windows the WinUSB path would find real devices
    monkeypatch.setenv("GS_USB_FD_BACKEND", "libusb")
    monkeypatch.setattr(device, "find_usb_devices", lambda: list(devs))
    monkeypatch.setattr(usb.util, "claim_interface", lambda dev, iface: dev.claim(iface))
    monkeypatch.setattr(usb.util, "release_interface", lambda *a, **k: None)
    monkeypatch.setattr(usb.util, "dispose_resources", lambda *a, **k: None)


@pytest.fixture
def fake(monkeypatch):
    """A fake gs_usb FD device (one interface, two channels) in place of the USB bus."""
    dev = FakeGsUsb()
    _install(monkeypatch, [dev])
    yield dev
    assert not device.GsUsbDevice._registry, "a bus was left open"
    assert not device._usb_refs, "a USB handle was left open"


@pytest.fixture
def fake_split(monkeypatch):
    """The tricore-debug-machine probe in its split layout: CAN1 on interface
    0, CAN2 on interface 4, one channel each."""
    dev = FakeGsUsb(interfaces=((0, 1), (4, 1)))
    _install(monkeypatch, [dev])
    yield dev
    assert not device.GsUsbDevice._registry, "a bus was left open"
    assert not device._usb_refs, "a USB handle was left open"


def pytest_collection_modifyitems(config, items):
    if os.environ.get("GS_USB_FD_HW") == "1":
        return
    skip = pytest.mark.skip(reason="hardware test: set GS_USB_FD_HW=1 with a probe on USB")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)
