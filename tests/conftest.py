import os

import pytest
import usb.util

from python_can_gs_usb_fd import device

from fake_device import FakeGsUsb


@pytest.fixture
def fake(monkeypatch):
    """A fake gs_usb FD device in place of the USB bus."""
    dev = FakeGsUsb()
    monkeypatch.setattr(device, "find_usb_devices", lambda: [dev])
    monkeypatch.setattr(usb.util, "claim_interface", lambda *a, **k: None)
    monkeypatch.setattr(usb.util, "release_interface", lambda *a, **k: None)
    monkeypatch.setattr(usb.util, "dispose_resources", lambda *a, **k: None)
    yield dev
    assert not device.GsUsbDevice._registry, "a bus was left open"


def pytest_collection_modifyitems(config, items):
    if os.environ.get("GS_USB_FD_HW") == "1":
        return
    skip = pytest.mark.skip(reason="hardware test: set GS_USB_FD_HW=1 with a probe on USB")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip)
