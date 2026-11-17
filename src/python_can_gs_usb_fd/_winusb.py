"""One interface of a composite USB device through WinUSB (Windows only).

(The same module as mcds_trace.winusb in the tricore-debug-machine repository.)

libusb opens a composite WinUSB device as a whole: it takes every interface's
WinUSB handle, and WinUSB gives out one handle per interface, so the first
program to open a device locks every other one out of all of its interfaces
- a CAN tool out of a debug interface, a debugger out of CAN.  This module opens only the
interface asked for: it finds the interface's own device path by its
DeviceInterfaceGUID (which the probe's MS OS 2.0 descriptors register) and
talks to it through winusb.dll.  Another program can hold any other
interface meanwhile.

`WinUsbInterface` reads and writes like a pyusb Device, so code written
against pyusb runs on it unchanged: read(ep, size_or_buffer, timeout),
write(ep, data, timeout) and ctrl_transfer(...), raising pyusb's
USBTimeoutError / USBError.  All I/O is overlapped, so one thread may wait on
an endpoint while another uses a different one.
"""

from __future__ import annotations

import array
import ctypes
import re
import sys
import uuid
from dataclasses import dataclass

try:  # pyusb's exception types, so callers keep catching what they catch
    from usb.core import USBError, USBTimeoutError
except ImportError:  # pragma: no cover
    class USBError(IOError):
        def __init__(self, strerror, error_code=None, errno=None):
            IOError.__init__(self, errno, strerror)
            self.backend_error_code = error_code

    class USBTimeoutError(USBError):
        pass

AVAILABLE = sys.platform == 'win32'

ERROR_ACCESS_DENIED = 5
ERROR_IO_PENDING = 997
ERROR_OPERATION_ABORTED = 995
ERROR_SEM_TIMEOUT = 121
WAIT_OBJECT_0, WAIT_TIMEOUT = 0, 0x102


@dataclass(frozen=True)
class InterfacePath:
    path: str          # the interface's device path, for CreateFile
    serial: str        # the device's iSerialNumber (from its instance ID)
    vid: int
    pid: int
    interface: int     # bInterfaceNumber (from "&mi_NN"), -1 if not composite


if AVAILABLE:
    from ctypes import wintypes as wt   # Windows only (fails on some other platforms)

    _ULONG_PTR = ctypes.c_size_t

    class _GUID(ctypes.Structure):
        _fields_ = [('Data1', wt.DWORD), ('Data2', wt.WORD), ('Data3', wt.WORD),
                    ('Data4', ctypes.c_ubyte * 8)]

    class _SP_DEVICE_INTERFACE_DATA(ctypes.Structure):
        _fields_ = [('cbSize', wt.DWORD), ('InterfaceClassGuid', _GUID), ('Flags', wt.DWORD),
                    ('Reserved', _ULONG_PTR)]

    class _SP_DEVINFO_DATA(ctypes.Structure):
        _fields_ = [('cbSize', wt.DWORD), ('ClassGuid', _GUID), ('DevInst', wt.DWORD),
                    ('Reserved', _ULONG_PTR)]

    class _OVERLAPPED(ctypes.Structure):
        _fields_ = [('Internal', _ULONG_PTR), ('InternalHigh', _ULONG_PTR), ('Offset', wt.DWORD),
                    ('OffsetHigh', wt.DWORD), ('hEvent', wt.HANDLE)]

    class _SETUP(ctypes.Structure):
        _pack_ = 1
        _fields_ = [('RequestType', ctypes.c_ubyte), ('Request', ctypes.c_ubyte),
                    ('Value', ctypes.c_ushort), ('Index', ctypes.c_ushort),
                    ('Length', ctypes.c_ushort)]

    _k32 = ctypes.WinDLL('kernel32', use_last_error=True)
    _setupapi = ctypes.WinDLL('setupapi', use_last_error=True)
    _cfgmgr = ctypes.WinDLL('cfgmgr32', use_last_error=True)
    _winusb = ctypes.WinDLL('winusb', use_last_error=True)

    _INVALID_HANDLE = ctypes.c_void_p(-1).value

    def _proto(fn, res, *args):
        fn.restype = res
        fn.argtypes = args
        return fn

    _GetClassDevs = _proto(_setupapi.SetupDiGetClassDevsW, ctypes.c_void_p,
                           ctypes.POINTER(_GUID), wt.LPCWSTR, wt.HWND, wt.DWORD)
    _EnumIfaces = _proto(_setupapi.SetupDiEnumDeviceInterfaces, wt.BOOL, ctypes.c_void_p,
                         ctypes.c_void_p, ctypes.POINTER(_GUID), wt.DWORD,
                         ctypes.POINTER(_SP_DEVICE_INTERFACE_DATA))
    _IfaceDetail = _proto(_setupapi.SetupDiGetDeviceInterfaceDetailW, wt.BOOL, ctypes.c_void_p,
                          ctypes.POINTER(_SP_DEVICE_INTERFACE_DATA), ctypes.c_void_p, wt.DWORD,
                          ctypes.POINTER(wt.DWORD), ctypes.POINTER(_SP_DEVINFO_DATA))
    _DestroyList = _proto(_setupapi.SetupDiDestroyDeviceInfoList, wt.BOOL, ctypes.c_void_p)
    _CM_Get_Parent = _proto(_cfgmgr.CM_Get_Parent, wt.DWORD, ctypes.POINTER(wt.DWORD), wt.DWORD,
                            wt.ULONG)
    _CM_Get_Device_ID = _proto(_cfgmgr.CM_Get_Device_IDW, wt.DWORD, wt.DWORD, wt.LPWSTR, wt.ULONG,
                               wt.ULONG)
    _CreateFile = _proto(_k32.CreateFileW, ctypes.c_void_p, wt.LPCWSTR, wt.DWORD, wt.DWORD,
                         ctypes.c_void_p, wt.DWORD, wt.DWORD, wt.HANDLE)
    _CloseHandle = _proto(_k32.CloseHandle, wt.BOOL, ctypes.c_void_p)
    _CreateEvent = _proto(_k32.CreateEventW, ctypes.c_void_p, ctypes.c_void_p, wt.BOOL, wt.BOOL,
                          wt.LPCWSTR)
    _Wait = _proto(_k32.WaitForSingleObject, wt.DWORD, ctypes.c_void_p, wt.DWORD)
    _CancelIoEx = _proto(_k32.CancelIoEx, wt.BOOL, ctypes.c_void_p, ctypes.POINTER(_OVERLAPPED))

    _WU_Initialize = _proto(_winusb.WinUsb_Initialize, wt.BOOL, ctypes.c_void_p,
                            ctypes.POINTER(ctypes.c_void_p))
    _WU_Free = _proto(_winusb.WinUsb_Free, wt.BOOL, ctypes.c_void_p)
    _WU_ReadPipe = _proto(_winusb.WinUsb_ReadPipe, wt.BOOL, ctypes.c_void_p, ctypes.c_ubyte,
                          ctypes.c_void_p, wt.ULONG, ctypes.POINTER(wt.ULONG),
                          ctypes.POINTER(_OVERLAPPED))
    _WU_WritePipe = _proto(_winusb.WinUsb_WritePipe, wt.BOOL, ctypes.c_void_p, ctypes.c_ubyte,
                           ctypes.c_void_p, wt.ULONG, ctypes.POINTER(wt.ULONG),
                           ctypes.POINTER(_OVERLAPPED))
    _WU_Control = _proto(_winusb.WinUsb_ControlTransfer, wt.BOOL, ctypes.c_void_p, _SETUP,
                         ctypes.c_void_p, wt.ULONG, ctypes.POINTER(wt.ULONG),
                         ctypes.POINTER(_OVERLAPPED))
    _WU_Result = _proto(_winusb.WinUsb_GetOverlappedResult, wt.BOOL, ctypes.c_void_p,
                        ctypes.POINTER(_OVERLAPPED), ctypes.POINTER(wt.DWORD), wt.BOOL)
    _WU_ResetPipe = _proto(_winusb.WinUsb_ResetPipe, wt.BOOL, ctypes.c_void_p, ctypes.c_ubyte)
    _WU_FlushPipe = _proto(_winusb.WinUsb_FlushPipe, wt.BOOL, ctypes.c_void_p, ctypes.c_ubyte)
    _WU_GetDescriptor = _proto(_winusb.WinUsb_GetDescriptor, wt.BOOL, ctypes.c_void_p,
                               ctypes.c_ubyte, ctypes.c_ubyte, ctypes.c_ushort, ctypes.c_void_p,
                               wt.ULONG, ctypes.POINTER(wt.ULONG))

    class _IF_DESC(ctypes.Structure):
        _pack_ = 1
        _fields_ = [('bLength', ctypes.c_ubyte), ('bDescriptorType', ctypes.c_ubyte),
                    ('bInterfaceNumber', ctypes.c_ubyte), ('bAlternateSetting', ctypes.c_ubyte),
                    ('bNumEndpoints', ctypes.c_ubyte), ('bInterfaceClass', ctypes.c_ubyte),
                    ('bInterfaceSubClass', ctypes.c_ubyte), ('bInterfaceProtocol', ctypes.c_ubyte),
                    ('iInterface', ctypes.c_ubyte)]

    class _PIPE_INFO(ctypes.Structure):
        _fields_ = [('PipeType', ctypes.c_int), ('PipeId', ctypes.c_ubyte),
                    ('MaximumPacketSize', ctypes.c_ushort), ('Interval', ctypes.c_ubyte)]

    _WU_QuerySettings = _proto(_winusb.WinUsb_QueryInterfaceSettings, wt.BOOL, ctypes.c_void_p,
                               ctypes.c_ubyte, ctypes.POINTER(_IF_DESC))
    _WU_QueryPipe = _proto(_winusb.WinUsb_QueryPipe, wt.BOOL, ctypes.c_void_p, ctypes.c_ubyte,
                           ctypes.c_ubyte, ctypes.POINTER(_PIPE_INFO))


def _guid(s: str):
    u = uuid.UUID(s)
    g = _GUID()
    g.Data1, g.Data2, g.Data3 = u.fields[0], u.fields[1], u.fields[2]
    g.Data4[:] = list(u.bytes[8:])
    return g


_ID_RE = re.compile(r'VID_([0-9A-F]{4})&PID_([0-9A-F]{4})\\(.*)$', re.I)
_MI_RE = re.compile(r'&mi_([0-9a-f]{2})', re.I)


def _device_id(devinst: int) -> str:
    buf = ctypes.create_unicode_buffer(512)
    if _CM_Get_Device_ID(devinst, buf, len(buf), 0) != 0:
        return ''
    return buf.value


def list_interfaces(guid: str, vid: int | None = None, pid: int | None = None) -> list[InterfacePath]:
    """The present interfaces registered under DeviceInterfaceGUID guid,
    optionally only those of vid:pid, sorted by serial and interface."""
    if not AVAILABLE:
        return []
    g = _guid(guid)
    h = _GetClassDevs(ctypes.byref(g), None, None, 0x02 | 0x10)   # PRESENT | DEVICEINTERFACE
    if h is None or h == _INVALID_HANDLE:
        return []
    out = []
    try:
        k = 0
        while True:
            ifd = _SP_DEVICE_INTERFACE_DATA()
            ifd.cbSize = ctypes.sizeof(ifd)
            if not _EnumIfaces(h, None, ctypes.byref(g), k, ctypes.byref(ifd)):
                break
            k += 1
            need = wt.DWORD(0)
            _IfaceDetail(h, ctypes.byref(ifd), None, 0, ctypes.byref(need), None)
            if not need.value:
                continue
            raw = ctypes.create_string_buffer(need.value)
            # SP_DEVICE_INTERFACE_DETAIL_DATA_W: cbSize is the fixed part (8 on 64-bit)
            ctypes.cast(raw, ctypes.POINTER(wt.DWORD))[0] = 8 if ctypes.sizeof(ctypes.c_void_p) == 8 else 6
            dev = _SP_DEVINFO_DATA()
            dev.cbSize = ctypes.sizeof(dev)
            if not _IfaceDetail(h, ctypes.byref(ifd), raw, need, None, ctypes.byref(dev)):
                continue
            path = ctypes.wstring_at(ctypes.addressof(raw) + 4)
            mi = _MI_RE.search(path)
            # the device (not interface) instance ID ends in the serial: a
            # composite function's parent is the device
            ident = _device_id(dev.DevInst)
            if mi:
                parent = wt.DWORD(0)
                if _CM_Get_Parent(ctypes.byref(parent), dev.DevInst, 0) == 0:
                    ident = _device_id(parent.value)
            m = _ID_RE.search(ident)
            if not m:
                continue
            v, p, serial = int(m.group(1), 16), int(m.group(2), 16), m.group(3)
            if (vid is not None and v != vid) or (pid is not None and p != pid):
                continue
            out.append(InterfacePath(path, serial, v, p, int(mi.group(1), 16) if mi else -1))
    finally:
        _DestroyList(h)
    return sorted(out, key=lambda x: (x.serial, x.interface))


class WinUsbInterface:
    """An open WinUSB interface, with pyusb Device-style I/O."""

    def __init__(self, where: InterfacePath | str):
        if not AVAILABLE:
            raise USBError('WinUSB is Windows only')
        self.info = where if isinstance(where, InterfacePath) else None
        path = where.path if isinstance(where, InterfacePath) else where
        self.serial_number = self.info.serial if self.info else None
        self._file = _CreateFile(path, 0xC0000000, 0x3, None, 3, 0x80 | 0x40000000, None)
        if self._file is None or self._file == _INVALID_HANDLE:
            err = ctypes.get_last_error()
            if err == ERROR_ACCESS_DENIED:
                raise USBError('interface busy: another program has it open', err, 16)
            raise USBError('cannot open %s (Windows error %d)' % (path, err), err)
        self._h = ctypes.c_void_p()
        if not _WU_Initialize(self._file, ctypes.byref(self._h)):
            err = ctypes.get_last_error()
            _CloseHandle(self._file)
            self._file = None
            raise USBError('WinUsb_Initialize failed (Windows error %d)' % err, err)

    # -- lifetime -----------------------------------------------------------------

    def close(self) -> None:
        if getattr(self, '_h', None) and self._h.value:
            _WU_Free(self._h)
            self._h = ctypes.c_void_p()
        if getattr(self, '_file', None):
            _CloseHandle(self._file)
            self._file = None

    def __enter__(self) -> 'WinUsbInterface':
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    # -- I/O ----------------------------------------------------------------------

    def _io(self, start, timeout: int | None) -> int:
        """Run one overlapped operation start(ov) to completion; the bytes
        moved. A timeout cancels it: what arrived before counts, nothing
        arrived raises USBTimeoutError."""
        if not self._h.value:
            raise USBError('interface closed')
        ov = _OVERLAPPED()
        ov.hEvent = _CreateEvent(None, True, False, None)
        try:
            if not start(ctypes.byref(ov)):
                err = ctypes.get_last_error()
                if err != ERROR_IO_PENDING:
                    raise USBError('USB transfer failed (Windows error %d)' % err, err)
            ms = 0xFFFFFFFF if not timeout else int(timeout)
            timed_out = _Wait(ov.hEvent, ms) == WAIT_TIMEOUT
            if timed_out:
                _CancelIoEx(self._file, ctypes.byref(ov))
            n = wt.DWORD(0)
            ok = _WU_Result(self._h, ctypes.byref(ov), ctypes.byref(n), True)
            if not ok:
                err = ctypes.get_last_error()
                if n.value:
                    return n.value
                if timed_out or err in (ERROR_OPERATION_ABORTED, ERROR_SEM_TIMEOUT):
                    raise USBTimeoutError('Operation timed out', err, 110)
                raise USBError('USB transfer failed (Windows error %d)' % err, err)
            return n.value
        finally:
            _CloseHandle(ov.hEvent)

    def read(self, endpoint: int, size_or_buffer, timeout: int | None = None):
        """pyusb's Device.read: an array('B') of what arrived, or, given a
        buffer, the count read into it."""
        if isinstance(size_or_buffer, int):
            buf = array.array('B', bytes(size_or_buffer))
            n = self._read_into(endpoint, buf, timeout)
            return buf[:n]
        return self._read_into(endpoint, size_or_buffer, timeout)

    def _read_into(self, endpoint, buf, timeout) -> int:
        n = len(buf) * getattr(buf, 'itemsize', 1)
        c = (ctypes.c_ubyte * n).from_buffer(buf)
        return self._io(lambda ov: _WU_ReadPipe(self._h, endpoint | 0x80, c, n, None, ov), timeout)

    def write(self, endpoint: int, data, timeout: int | None = None) -> int:
        b = bytes(data)
        c = ctypes.create_string_buffer(b, len(b))
        return self._io(lambda ov: _WU_WritePipe(self._h, endpoint & 0x7F, c, len(b), None, ov),
                        timeout)

    def ctrl_transfer(self, bmRequestType: int, bRequest: int, wValue: int = 0, wIndex: int = 0,
                      data_or_wLength=None, timeout: int | None = None):
        """pyusb's Device.ctrl_transfer: IN returns an array('B'), OUT the
        count written. WinUSB puts the interface's own number in wIndex for
        interface requests."""
        if bmRequestType & 0x80:
            n = int(data_or_wLength or 0)
            buf = (ctypes.c_ubyte * max(n, 1))()
        else:
            b = bytes(data_or_wLength or b'')
            n = len(b)
            buf = (ctypes.c_ubyte * max(n, 1)).from_buffer_copy(b or b'\0')
        setup = _SETUP(bmRequestType, bRequest, wValue & 0xFFFF, wIndex & 0xFFFF, n)
        got = self._io(lambda ov: _WU_Control(self._h, setup, buf, n, None, ov), timeout)
        if bmRequestType & 0x80:
            return array.array('B', bytes(buf)[:got])
        return got

    def clear_halt(self, endpoint: int) -> None:
        _WU_ResetPipe(self._h, endpoint)

    def flush(self, endpoint: int) -> None:
        """Drop what WinUSB buffered for an IN endpoint."""
        _WU_FlushPipe(self._h, endpoint | 0x80)

    def string(self, index: int, langid: int = 0x0409) -> str:
        buf = (ctypes.c_ubyte * 256)()
        n = wt.ULONG(0)
        if not index or not _WU_GetDescriptor(self._h, 3, index, langid, buf, 256, ctypes.byref(n)):
            return ''
        return bytes(buf)[2:n.value].decode('utf-16-le', errors='replace')

    def pipes(self) -> list[tuple[int, int, int]]:
        """The interface's endpoints: (address, type 0..3, max packet size)."""
        d = _IF_DESC()
        if not _WU_QuerySettings(self._h, 0, ctypes.byref(d)):
            raise USBError('no interface settings (Windows error %d)' % ctypes.get_last_error())
        out = []
        for k in range(d.bNumEndpoints):
            pi = _PIPE_INFO()
            if _WU_QueryPipe(self._h, 0, k, ctypes.byref(pi)):
                out.append((pi.PipeId, pi.PipeType, pi.MaximumPacketSize))
        return out

    def device_descriptor(self) -> bytes:
        buf = (ctypes.c_ubyte * 18)()
        n = wt.ULONG(0)
        if not _WU_GetDescriptor(self._h, 1, 0, 0, buf, 18, ctypes.byref(n)):
            raise USBError('no device descriptor (Windows error %d)' % ctypes.get_last_error())
        return bytes(buf)[:n.value]
