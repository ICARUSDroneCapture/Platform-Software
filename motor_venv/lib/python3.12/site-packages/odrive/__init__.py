
import os
import sys

# We want to use the fibre package that is included with the odrive package
# in order to avoid any version mismatch issues,
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "pyfibre"))

import asyncio
import concurrent
import threading
import time
from typing import Optional

from .version import __version__
from .utils import get_serial_number_str, get_serial_number_str_sync, attach_metadata

# Backwards compatibility with Python 3.6 (default on Ubuntu 18.04)
if sys.version_info < (3, 7):
    asyncio.get_running_loop = asyncio.get_event_loop

    def asyncio_run(coro):
        loop = asyncio.get_event_loop()
        return loop.run_until_complete(coro)
    asyncio.run = asyncio_run

default_usb_search_path = 'usb:idVendor=0x1209,idProduct=0x0D32,bInterfaceClass=0,bInterfaceSubClass=1,bInterfaceProtocol=0'
default_search_path = default_usb_search_path


_discovery_lock = threading.Lock()
_discovery_started = [False]
_discovery_path = [None]

connected_devices = []
connected_devices_changed = concurrent.futures.Future()

def start_discovery(path):
    """
    Starts device discovery in a background thread. This function returns
    immediately.
    If discovery was already started, this function does nothing.
    """

    # Start backend if it's not already started
    with _discovery_lock:
        if _discovery_started[0]:
            if path != _discovery_path[0]:
                raise Exception("Cannot change discovery path between multiple find_any() "
                                "calls: {} != {}. Use fibre.Domain() directly for finer "
                                "grained discovery control.".format(path, _discovery_path))
            return # discovery already started
        _discovery_started[0] = True
        _discovery_path[0] = path

    async def discovered_object(obj):
        def lost_object(_):
            connected_devices.remove(obj)

            # indicate that connected_devices changed
            global connected_devices_changed
            signal = connected_devices_changed
            connected_devices_changed = concurrent.futures.Future()
            signal.set_result(None)

        await attach_metadata(obj)

        with _discovery_lock:
            connected_devices.append(obj)
        obj._on_lost.add_done_callback(lost_object)

        # indicate that connected_devices changed
        global connected_devices_changed
        signal = connected_devices_changed
        connected_devices_changed = concurrent.futures.Future()
        signal.set_result(None)

    def domain_thread():
        _domain_termination_token = concurrent.futures.Future() # unused

        import fibre
        with fibre.Domain(path) as domain:
            discovery = domain.run_discovery(discovered_object)
            _domain_termination_token.result()
            discovery.stop()

    threading.Thread(target=domain_thread, daemon=True).start()


def find_any(path: str = default_search_path,
             serial_number: str = None,
             cancellation_token: concurrent.futures.Future = None,
             timeout: float = None):
    """
    Blocks until the first matching ODrive object is connected and then returns
    that object.

    If find_any() is called multiple times, the same object may be returned
    (depending on the serial_number argument).

    The first call to find_any() will start a background thread that handles
    the backend. This background thread will keep running until the program is
    terminated.
    
    If you want finer grained control over object discovery
    consider using fibre.Domain directly.
    """
    assert cancellation_token is None or isinstance(cancellation_token, concurrent.futures.Future)
    if cancellation_token is None:
        cancellation_tokens = []
    else:
        cancellation_tokens = [cancellation_token]

    start_discovery(path)

    wait_start = time.monotonic()
    while True:
        signal = connected_devices_changed
        with _discovery_lock:
            for obj in connected_devices:
                if serial_number is None or obj._serial_number == serial_number:
                    return obj
        
        current_timeout = None if timeout is None else max(0, timeout - (time.monotonic() - wait_start))
        wait_result = concurrent.futures.wait([signal] + cancellation_tokens, timeout=current_timeout, return_when=concurrent.futures.FIRST_COMPLETED)
        
        if (not cancellation_token is None) and (cancellation_token in wait_result.done):
            raise concurrent.futures.CancelledError()
        elif (not signal in wait_result.done):
            raise TimeoutError()


async def find_any_async(path: str = default_search_path, serial_number: Optional[str] = None):
    start_discovery(path)

    while True:
        signal = connected_devices_changed
        with _discovery_lock:
            for obj in connected_devices:
                if serial_number is None or obj._serial_number == serial_number:
                    return obj
        await asyncio.shield(asyncio.wrap_future(signal))
