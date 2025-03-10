import asyncio
import ctypes
from dataclasses import dataclass
import os
import platform
import sys
import threading
from typing import Callable, Dict, List, Optional, Tuple
from .hw_version import HwVersion

_lib_names = {
    ('Linux', 'x86_64'): 'libodrive-linux-x86_64.so',
    ('Linux', 'aarch64'): 'libodrive-linux-aarch64.so',
    ('Windows', 'AMD64'): 'libodrive-windows-x64.dll',
    ('Darwin', 'x86_64'): 'libodrive-macos-x86_64.dylib',
    ('Darwin', 'arm64'): 'libodrive-macos-arm64.dylib',
}

class _HwVersion(ctypes.Structure):
    _fields_ = [("product_line", ctypes.c_uint8),
                ("version", ctypes.c_uint8),
                ("variant", ctypes.c_uint8),
                ("reserved", ctypes.c_uint8)]

class _FwManifest(ctypes.Structure):
    _fields_ = [("magic_number", ctypes.c_uint32),
                ("fw_version_major", ctypes.c_uint8),
                ("fw_version_minor", ctypes.c_uint8),
                ("fw_version_revision", ctypes.c_uint8),
                ("fw_version_unreleased", ctypes.c_uint8),
                ("hw_version", _HwVersion),
                ("reserved", ctypes.c_uint8 * 32),
                ("build", ctypes.c_uint8 * 20)]


class LibODrive():
    """
    This class is not thread-safe.
    
    All public member functions should be called from the same thread as `loop`
    and all callbacks are called on `loop`.

    Internally, this class launches a separate I/O thread on which most of
    libodrive's backend actually runs. Libodrive handles event passing between
    the two threads.
    """

    @staticmethod
    def get_default_lib_path():
        _system_desc = (platform.system(), platform.machine())

        if _system_desc == ('Linux', 'aarch64') and sys.maxsize <= 2**32:
            _system_desc = ('Linux', 'armv7l') # Python running in 32-bit mode on 64-bit OS

        if not _system_desc in _lib_names:
            raise ModuleNotFoundError("libodrive is not supported on your platform ({} {}).")

        _script_dir = os.path.dirname(os.path.realpath(__file__))
        return os.path.join(_script_dir, "lib", _lib_names[_system_desc])

    @property
    def version(self):
        return (
            (self._version.value >> 24) & 0xff,
            (self._version.value >> 16) & 0xff,
            (self._version.value >> 8) & 0xff,
            (self._version.value >> 0) & 0xff,
        )

    def __init__(self, loop: asyncio.AbstractEventLoop, lib_path: Optional[str] = None):
        self._loop: asyncio.AbstractEventLoop = asyncio.get_running_loop() if loop is None else loop

        if lib_path is None:
            lib_path = LibODrive.get_default_lib_path()
            if not os.path.isfile(lib_path):
                raise ImportError(f"{lib_path} not found. Try to reinstall the Python package. If you're a developer, run tools/setup.sh first.")

        if os.name == 'nt':
            dll_dir = os.path.dirname(lib_path)
            if sys.version_info >= (3, 8):
                os.add_dll_directory(dll_dir)
            else:
                os.environ['PATH'] = dll_dir + os.pathsep + os.environ['PATH']
            self._lib = ctypes.windll.LoadLibrary(lib_path)
        else:
            self._lib = ctypes.cdll.LoadLibrary(lib_path)

        self._version = ctypes.c_uint32.in_dll(self._lib, 'libodrive_version')
        if self._version.value & 0xffff0000 != 0x00080000:
            raise ImportError(f"Incompatible libodrive version ({self._version.value:08X}). Try to reinstall the Python package. If you're a developer, run tools/setup.sh first.")
    
        # Hack to allow coexistence with libfibre.
        # On Windows, libusb_open() can only be called once per device and process
        # (macOS doesn't have this limitation), so we must ignore runtime devices in
        # libodrive to prevent interference with libfibre until odrivetool is fully
        # migrated. See also handled_by_libfibre() in usb_discoverer.cpp.
        # This is a runtime (rather than compile-time) setting because the GUI is
        # already purely libodrive based.
        try:
            ctypes.c_uint32.in_dll(self._lib, 'libodrive_ignore_runtime_odrives').value = 1
        except: # symbol not present in libodrive 0.7.0 or earlier
            print("Warning: this version of libodrive can't coexist with libfibre")

        # Load functions
        self._lib.libodrive_init.argtypes = []
        self._lib.libodrive_init.restype = ctypes.c_void_p

        self._lib.libodrive_deinit.argtypes = [ctypes.c_void_p]
        self._lib.libodrive_deinit.restype = None

        self._lib.libodrive_iteration.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._lib.libodrive_iteration.restype = ctypes.c_int

        self._lib.libodrive_interrupt_iteration.argtypes = [ctypes.c_void_p]
        self._lib.libodrive_interrupt_iteration.restype = ctypes.c_int

        self._lib.libodrive_handle_callbacks.argtypes = [ctypes.c_void_p]
        self._lib.libodrive_handle_callbacks.restype = ctypes.c_int

        self._lib.libodrive_start_usb_discovery.argtypes = [ctypes.c_void_p, _TOnFoundDevice, _TOnLostDevice, ctypes.c_void_p]
        self._lib.libodrive_start_usb_discovery.restype = ctypes.c_void_p

        self._lib.libodrive_stop_discovery.argtypes = [ctypes.c_void_p]
        self._lib.libodrive_stop_discovery.restype = ctypes.c_int

        self._lib.libodrive_usb_device_from_handle.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self._lib.libodrive_usb_device_from_handle.restype = None

        self._lib.libodrive_connect.argtypes = [ctypes.c_void_p, _TOnConnected, _TOnConnectionFailed]
        self._lib.libodrive_connect.restype = ctypes.c_int

        self._lib.libodrive_disconnect.argtypes = [ctypes.c_void_p]
        self._lib.libodrive_disconnect.restype = ctypes.c_int

        self._lib.libodrive_start_installation.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, _TOnInstallationProgress, _TOnInstallationDone, ctypes.c_void_p]
        self._lib.libodrive_start_installation.restype = ctypes.c_int

        self._lib.libodrive_open_firmware.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.POINTER(_FwManifest))]
        self._lib.libodrive_open_firmware.restype = ctypes.c_int

        self._lib.libodrive_close_firmware.argtypes = [ctypes.c_void_p]
        self._lib.libodrive_close_firmware.restype = None

        # Init
        self._ctx = self._lib.libodrive_init()
        assert self._ctx

        self._notify_handle: Optional[asyncio.Handle] = None
        self._worker_thread_shutdown = False
        self._worker_thread = threading.Thread(target=self._worker_thread_func)
        self._notify_pending = False
        #self._worker_thread.start()

    def stop_thread(self):
        self._worker_thread_shutdown = True
        self._lib.libodrive_interrupt_iteration(self._ctx)
        self._worker_thread.join()
        if not self._notify_handle is None:
            self._notify_handle.cancel()

    def deinit(self):
        self._lib.libodrive_deinit(self._ctx)

    def _notify(self):
        self._notify_pending = False
        self._lib.libodrive_handle_callbacks(self._ctx)

    def _worker_thread_func(self):
        while not self._worker_thread_shutdown:
            result = self._lib.libodrive_iteration(self._ctx, -1) # no timeout
            assert result == 0 or result == 1
            if result == 1 and not self._notify_pending:
                self._notify_pending = True
                self._notify_handle = self._loop.call_soon_threadsafe(self._notify)

    def start_usb_discovery(self):
        def _enter(intf: Interface):
            handle = self._lib.libodrive_start_usb_discovery(self._ctx, _on_found_device, _on_lost_device, id(intf))
            assert handle
            intf._handle = handle
            _intf_map[id(intf)] = intf # callbacks only called on same thread, so it's safe to assign this after starting
            return intf
        intf = Interface(self._lib, _enter)
        return intf

    def open_firmware(self, data: bytes):
        handle = ctypes.c_void_p()
        manifest = ctypes.POINTER(_FwManifest)()
        self._lib.libodrive_open_firmware(data, len(data), ctypes.byref(handle), ctypes.byref(manifest))
        assert handle
        assert manifest
        return Firmware(self._lib, handle, manifest.contents)

class Interface():
    @staticmethod
    def _on_found_device(udata: int, device: int, serial_number: bytes, product_string: bytes, fibre2_capable: bool, handle: int, msg: bytes):
        intf = _intf_map[udata]
        py_dev = Device(intf._lib, device, serial_number.decode(), product_string.decode(), fibre2_capable)
        _dev_map[device] = py_dev
        intf.devices.append(py_dev)
        
        # signal all waiting tasks and recycle _event for next round
        intf._event.set()
        intf._event = asyncio.Event()

    @staticmethod
    def _on_lost_device(udata: int, device: int):
        intf = _intf_map[udata]
        py_dev = _dev_map.pop(device)
        intf.devices.remove(py_dev)

    def __init__(self, lib, enter_impl):
        self._lib = lib
        self._enter_impl = enter_impl
        self._handle = 0 # set in start_usb_discovery()
        self.devices: List[Device] = []
        self._event = asyncio.Event()

    def __enter__(self):
        return self._enter_impl(self)
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        _intf_map.pop(id(self))
        self._lib.libodrive_stop_discovery(self._handle)
        self._handle = 0

    def get_device(self, serial_number: Optional[str], is_bootloader: Optional[bool]):
        for dev in self.devices:
            if not serial_number is None and dev.serial_number != serial_number:
                continue
            if not is_bootloader is None and dev.is_bootloader != is_bootloader:
                continue
            return dev
        return None

    async def wait_for(self, serial_number: Optional[str], is_bootloader: Optional[bool]):
        while True:
            dev = self.get_device(serial_number, is_bootloader)
            if not dev is None:
                return dev
            await self._event.wait() # wait for another change in the set of known devices


class Device():
    def __init__(self, lib, handle: int, serial_number: str, product_string: str, is_bootloader: bool):
        self._lib = lib
        self._handle = handle
        self.serial_number = serial_number
        self.product_string = product_string
        self.is_bootloader = is_bootloader
        self.hw_version = None
        self._connection_future: asyncio.Future = asyncio.Future()
        self._on_installation_progress_cb: Optional[Callable[[bool, str, int, int], None]] = None

    @staticmethod
    def _on_connected(device: int, hw_version, manufacturer: bytes):
        _dev_map[device].hw_version = HwVersion.from_tuple((hw_version.contents.product_line, hw_version.contents.version, hw_version.contents.variant))
        _dev_map[device]._connection_future.set_result(None)

    @staticmethod
    def _on_connection_failed(device: int, msg: bytes):
        # This can be called when the connection has already be established.
        if not _dev_map[device]._connection_future.done():
            _dev_map[device]._connection_future.set_exception(Exception(msg.decode()))

    @staticmethod
    def _on_installation_progress(device: int, new_action_group: bool, action_string: bytes, action_index: int, n_actions: int):
        _dev_map[device]._on_installation_progress_cb(new_action_group, action_string.decode(), action_index, n_actions)

    @staticmethod
    def _on_installation_done(device: int, msg: bytes):
        if len(msg) == 0:
            _dev_map[device]._installation_done.set_result(None)
        else:
            _dev_map[device]._installation_done.set_exception(Exception(msg.decode()))

    async def connect_bootloader(self) -> None:
        assert self._lib.libodrive_connect(self._handle, _on_connected, _on_connection_failed) == 0
        await self._connection_future

    async def run_installation(self, fw: 'Firmware', erase_all: bool, on_installation_progress: Callable[[bool, str, int, int], None]) -> None:
        self._installation_done: asyncio.Future[None] = asyncio.Future()
        self._on_installation_progress_cb = on_installation_progress
        assert self._lib.libodrive_start_installation(self._handle, fw._handle, erase_all, _on_installation_progress, _on_installation_done, self._handle) == 0
        await self._installation_done


class Firmware():
    def __init__(self, lib, handle: ctypes.c_void_p, manifest: _FwManifest):
        self._lib = lib
        self._handle = handle
        self._manifest = manifest

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self._lib.libodrive_close_firmware(self._handle)

    @property
    def fw_version(self) -> Tuple[int, int, int]:
        return (
            self._manifest.fw_version_major,
            self._manifest.fw_version_minor,
            self._manifest.fw_version_revision,
        )

    @property
    def hw_version(self) -> HwVersion:
        return HwVersion.from_tuple((
            self._manifest.hw_version.product_line,
            self._manifest.hw_version.version,
            self._manifest.hw_version.variant,
        ))

    @property
    def build(self) -> bytes:
        return bytes(self._manifest.build)


_TOnFoundDevice = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_bool, ctypes.c_void_p, ctypes.c_char_p)
_TOnLostDevice = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
_TOnConnected = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(_HwVersion), ctypes.c_void_p)
_TOnConnectionFailed = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_char_p)
_TOnInstallationProgress = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_bool, ctypes.c_char_p, ctypes.c_int, ctypes.c_int)
_TOnInstallationDone = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_char_p)

# Must keep references to FFI-wrapped callbacks to keep them alive
_on_found_device = _TOnFoundDevice(Interface._on_found_device)
_on_lost_device = _TOnLostDevice(Interface._on_lost_device)
_on_connected = _TOnConnected(Device._on_connected)
_on_connection_failed = _TOnConnectionFailed(Device._on_connection_failed)
_on_installation_progress = _TOnInstallationProgress(Device._on_installation_progress)
_on_installation_done = _TOnInstallationDone(Device._on_installation_done)

_dev_map: Dict[int, Device] = {}
_intf_map: Dict[int, Interface] = {}
