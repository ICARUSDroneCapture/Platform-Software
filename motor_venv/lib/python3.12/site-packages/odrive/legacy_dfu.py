#!/usr/bin/env python
"""
Tool for flashing .hex files to the ODrive via the STM built-in USB DFU mode.
"""

import aiohttp
import asyncio
import concurrent
import threading
from typing import Iterator, List, Tuple, Optional
import platform
import os
import usb.core
import fibre
import odrive
import odrive.legacy_config
import odrive.firmware
from odrive.utils import OperationAbortedException
from odrive.dfuse import *
from odrive.hw_version import HwVersion
import odrive.release_api as release_api
from odrive.release_api import VersionRelationship, format_version
from .dfu import DfuError, get_firmware

class ODriveInDfuMode(DfuDevice):
    def __init__(self, usbdev):
        DfuDevice.__init__(self, usbdev)
        self.board = None
        self._initialized = False

    def init(self, logger, ask):
        if self._initialized:
            return
        self._initialized = True
        DfuDevice.init(self)

        is_odrive3 = ('OTP Memory' in self.memories) and (self.memories['OTP Memory']['sectors'][0]['addr'] == 0x1fff7800)
        if is_odrive3:
            otp_sectors = self.memories['OTP Memory']['sectors']
        else:
            otp_sectors = None

        if logger._verbose and is_odrive3:
            logger.debug("OTP:")

            self.select_memory('OTP Memory')
            self.clear_status()

            # 512 Byte OTP
            otp_sector = [s for s in otp_sectors if s['addr'] == 0x1fff7800][0]
            data = self.read_sector(otp_sector)
            logger.debug(' '.join('{:02X}'.format(x) for x in data))

            # 16 lock bytes
            otp_lock_sector = [s for s in otp_sectors if s['addr'] == 0x1fff7A00][0]
            data = self.read_sector(otp_lock_sector)
            logger.debug(' '.join('{:02X}'.format(x) for x in data))

        if is_odrive3:
            # Reads the hardware version from one-time-programmable memory.
            # This is written on all ODrives sold since Summer 2018.
            otp_sectors = self.memories['OTP Memory']['sectors']
            otp_sector = [s for s in otp_sectors if s['addr'] == 0x1fff7800][0]
            self.select_memory('OTP Memory')
            self.clear_status()
            otp_data = self.read_sector(otp_sector)
            if otp_data[0] == 0:
                otp_data = otp_data[16:]
            if otp_data[0] == 0xfe:
                self.board = HwVersion(otp_data[3], otp_data[4], otp_data[5])
            else:
                self.board = HwVersion(3, 0, 0)
        else:
            if ask:
                choices = [HwVersion(4, 4, 58), HwVersion(5, 2, 0), HwVersion(6, 1, 0), HwVersion(6, 2, 0)]
                logger.info('Hardware version detection not supported in legacy DFU mode. What ODrive is this?')
                logger.warn('Make sure to select the correct version. Choosing the wrong version will cause undefined behavior and requires the DFU switch and legacy DFU mode to recover.')
                choice = odrive.utils.multiple_choice(                    
                    "ODrive version",
                    [v.display_name for v in choices]
                )
                self.board = choices[choice]
            else:
                self.board = None


class DfuDeviceDiscovery():
    def __init__(self):
        self._executor = None
        self._usb_lock = threading.Lock()

    def _get_devices(self):
        # Note that this function can interfere with DFU if run at the same time

        try:
            import libusb_package
            backend = libusb_package.get_libusb1_backend()
        except ModuleNotFoundError:
            backend = None # use PyUSB discovery

        with self._usb_lock:
            all_devices = {}
            for dev in usb.core.find(idVendor=0x0483, idProduct=0xdf11, find_all=True, backend=backend):
                try:
                    serial_number = dev.serial_number
                except ValueError:
                    print("found device but could not check serial number (retrying in 1s)")
                    continue
                all_devices[serial_number] = ODriveInDfuMode(dev)
        return all_devices

    async def __aenter__(self):
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=1).__enter__()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self._executor.__exit__(exc_type, exc_val, exc_tb)

    async def discover(self, serial_number: str):
        while True:
            devices = await asyncio.get_running_loop().run_in_executor(self._executor, self._get_devices)

            if serial_number is None and len(devices) > 0:
                return next(iter(devices.values()))
            if (not serial_number is None) and (serial_number in devices):
                return devices[serial_number]

            await asyncio.sleep(1)


async def enter_dfu_mode(device, dfu_discoverer: DfuDeviceDiscovery) -> ODriveInDfuMode:
    """
    Puts the specified device into (legacy) DFU mode.
    """
    if not hasattr(device, "enter_dfu_mode"):
        raise Exception(
            "The firmware on device {:08X} cannot soft enter DFU mode.\n"
            "Please remove power, put the DFU switch into DFU mode,\n"
            "then apply power again. Then try again.\n"
            "If it still doesn't work, you can try to use the DeFuse app or \n"
            "dfu-util, see the odrive documentation.\n"
            "You can also flash the firmware using STLink (`make flash`)"
            .format(device.serial_number)
        )

    serial_number = "{:08X}".format(device.serial_number)
    print("Putting device {:08X} into DFU mode...".format(device.serial_number))

    # If the new DFU system is already installed, need to disable bootloader to
    # be able to enter legacy DFU mode.
    # A warning will have been shown earlier in this case.
    if hasattr(device, 'disable_bootloader'):
        device.disable_bootloader()

    try:
        result = device.enter_dfu_mode()
    except fibre.ObjectLostError:
        result = True # this is expected because the device reboots
    if not result:
        raise DfuError("Could not put device into legacy DFU mode.")

    dfu_device = dfu_discoverer.discover(serial_number)

    if platform.system() == "Windows":
        async with _show_deferred_message(10, "Still waiting for the device to reappear.\n" "Use the Zadig utility to set the driver of 'STM32 BOOTLOADER' to libusb-win32."):
            return await dfu_device
    else:
        return await dfu_device


async def write_firmware(device, firmware: odrive.firmware.FirmwareFile, dfu_discoverer: DfuDeviceDiscovery, erase_all: bool, logger, installing_bootloader: bool):
    """
    Puts the device into DFU mode (if it's not already in DFU mode), writes
    the specified firmware file and then takes the device out of DFU mode.

    erase_all: If True, the entire flash memory is erased, including NVM.
               If False, only sectors that are overwritten is erased.
    """
    
    if not isinstance(device, ODriveInDfuMode):
        device = await enter_dfu_mode(device, dfu_discoverer)

    device.init(logger, ask=False)
    if device.board == HwVersion(3, 0, 0):
        # Jump to application
        device.jump_to_application(0x08000000)
        raise DfuError(
            "Could not determine hardware version. Flashing precompiled "
             "firmware could lead to unexpected results. Please use an "
             "STLink/2 to force-update the firmware anyway. Refer to "
             "https://docs.odriverobotics.com/developer-guide for details.")

    logger.debug("Memories on device: ")
    for k, mem in device.memories.items():
        logger.debug("{} sectors:".format(k))
        for sector in mem['sectors']:
            logger.debug(" {:08X} to {:08X}".format(
                sector['addr'],
                sector['addr'] + sector['len'] - 1))

    # fill sectors with data
    sections = list(firmware.get_flash_sections())
    for name, addr, content in sections:
        logger.debug(f"loading section {name} to 0x{addr:08x} ... 0x{(addr+len(content)):08x}")

    touched_sectors = list(_populate_sectors(device.memories['Internal Flash']['sectors'], sections))
    logger.debug("The following sectors will be flashed: ")
    for sector,_ in touched_sectors:
        logger.debug(" {:08X} to {:08X}".format(sector['addr'], sector['addr'] + sector['len'] - 1))

    device.select_memory('Internal Flash')
    device.clear_status()

    # Erase
    try:
        internal_flash_sectors = device.memories['Internal Flash']['sectors']
        if erase_all:
            erase_sectors = internal_flash_sectors
        else:
            erase_sectors = [s for s, d in touched_sectors]
        for i, sector in enumerate(erase_sectors):
            print("Erasing... (sector {}/{})  \r".format(i, len(erase_sectors)), end='', flush=True)
            device.erase_sector(sector)
        print('Erasing... done            \r', end='', flush=True)
    finally:
        print('', flush=True)

    # Flash
    try:
        for i, (sector, data) in enumerate(touched_sectors):
            print("Flashing... (sector {}/{})  \r".format(i, len(touched_sectors)), end='', flush=True)
            device.write_sector(sector, data)
        print('Flashing... done            \r', end='', flush=True)
    finally:
        print('', flush=True)

    # Verify
    try:
        for i, (sector, expected_data) in enumerate(touched_sectors):
            print("Verifying... (sector {}/{})  \r".format(i, len(touched_sectors)), end='', flush=True)
            observed_data = device.read_sector(sector)
            mismatch_pos = _get_first_mismatch_index(observed_data, expected_data)
            if not mismatch_pos is None:
                mismatch_pos -= mismatch_pos % 16
                observed_snippet = ' '.join('{:02X}'.format(x) for x in observed_data[mismatch_pos:mismatch_pos+16])
                expected_snippet = ' '.join('{:02X}'.format(x) for x in expected_data[mismatch_pos:mismatch_pos+16])
                raise RuntimeError("Verification failed around address 0x{:08X}:\n".format(sector['addr'] + mismatch_pos) +
                                   "  expected: " + expected_snippet + "\n"
                                   "  observed: " + observed_snippet)
        print('Verifying... done            \r', end='', flush=True)
    finally:
        print('', flush=True)

    # Jump to application
    device.jump_to_application(0x08000000)


def unlock_device(serial_number, cancellation_token):
    # TODO: this function is outdated
    
    print("Looking for ODrive in DFU mode...")
    print("If the program hangs at this point, try to set the DFU switch to \"DFU\" and power cycle the ODrive.")

    stm_device = find_device_in_dfu_mode(serial_number, cancellation_token)
    dfudev = DfuDevice(stm_device)

    print("Unlocking device (this may take a few seconds)...")
    dfudev.unprotect()
    print("done")
    print("")
    print("Now do the following:")
    print(" 1. Put the DFU switch on the ODrive to \"DFU\"")
    print(" 2. Power-cycle the ODrive")
    print(" 3. Run \"odrivetool dfu\" (or any third party DFU tool)")
    print(" 4. Put the DFU switch on the ODrive to \"RUN\"")


async def launch_dfu(serial_number: Optional[str], path: Optional[str], channel: Optional[str], version: Optional[str], erase_all: bool, logger, force: bool = False, installing_bootloader: bool = False, release_type: str = 'firmware'):
    """
    Runs the complete interactive DFU process:

    1. Wait for device in either DFU mode or normal mode. If `serial_number` is
       None, the first discovered device is selected, otherwise only the
       specified device is accepted.

    2. If `path` is None, check for the latest firmware, present it to the user
       and ask whether to continue. Otherwise don't ask and always continue.

    3. If the device is in normal mode, put it into DFU mode.

    4. Write flash memory.

    5. Exit DFU mode.

    path: Path to a .elf path or None to check online.
    channel: Channel on which to check for firmware (master, devel, ...)
    version: Exact firmware version
    """

    assert sum([bool(path), bool(channel), bool(version)]) == 1

    async with DfuDeviceDiscovery() as dfu_discoverer:
        logger.info("Waiting for ODrive...")

        # Wait for device either in DFU mode or in normal mode, whichever is
        # found first.
        done, pending = await asyncio.wait([
            asyncio.create_task(odrive.find_any_async(serial_number=serial_number)),
            asyncio.create_task(dfu_discoverer.discover(serial_number=serial_number))
        ], return_when=asyncio.FIRST_COMPLETED)
        for t in pending:
            t.cancel()
            try:
                await t
            except asyncio.CancelledError:
                pass
        device = (await asyncio.gather(*done))[0]

        found_in_dfu = isinstance(device, ODriveInDfuMode)

        if not found_in_dfu:
            current_fw_version = (
                (device.fw_version_major, device.fw_version_minor, device.fw_version_revision)
                if hasattr(device, "fw_version_major") and hasattr(device, "fw_version_minor") and hasattr(device, "fw_version_revision")
                else (0, 0, 0)
            )

            hw_version_major = device.hw_version_major if hasattr(device, "hw_version_major") else 0

            buggy_firmwares = {(0, 6, 5), (0, 6, 6)}

            if hw_version_major == 0 or current_fw_version == (0, 0, 0) or (hw_version_major == 4 and (current_fw_version in buggy_firmwares)):
                raise DfuError(
                    "The firmware on device {:08X} does not support starting a firmware update from\n"
                    "RUN mode. Please remove power, put the DFU switch into DFU mode, then apply power\n"
                    "again. Then try again.\n"
                    "Refer to https://docs.odriverobotics.com/v/latest/guides/firmware-update.html for details."
                    .format(device.serial_number)
                )

            has_new_dfu = hasattr(device, 'bootloader_version') and device.bootloader_version >= 0x00010000
            if has_new_dfu and installing_bootloader and not force:
                if not odrive.utils.yes_no_prompt("Bootloader is already installed in this device. Do you want to re-install it?", True):
                    return
            elif has_new_dfu and not force:
                if not odrive.utils.yes_no_prompt("This device is set up to use the new DFU system but you're running the legacy DFU command. "
                     "Try `odrivetool new-dfu` or https://gui.odriverobotics.com/dfu instead. "
                     "Do you want to proceed anyway with the legacy DFU system?", False):
                    return

        if path:
            assert os.path.isfile(path)
            assert channel is None
            file = odrive.firmware.FirmwareFile.from_file(path)
        else:
            if isinstance(device, ODriveInDfuMode):
                device.init(logger, ask=True)
            board = device.board if isinstance(device, ODriveInDfuMode) else device._board
            build_id_short: Optional[str] = "{:08x}".format(device.commit_hash) if hasattr(device, 'commit_hash') else None
            file = await get_firmware(board, build_id_short, channel, version, logger, not force, release_type)

        # Config erased anyway
        #may_have_config = found_in_dfu or (hasattr(device, 'user_config_loaded') and device.user_config_loaded)
        #if may_have_config and not force:
        #    if not odrive.utils.yes_no_prompt("The device may have user configuration that will be lost after the firmware upgrade. If you want to back this up please run `odrivetool backup-config` first. Do you want to continue anyway?", True):
        #        return

        await write_firmware(device, file, dfu_discoverer, erase_all, logger, installing_bootloader)

        if installing_bootloader:
            logger.success("Bootloader upload successful.")
            logger.info("To complete the setup, run `odrivetool new-dfu` or go to https://gui.odriverobotics.com/dfu.")
        elif not found_in_dfu:
            logger.info("Waiting for the device to reappear...")
            device = await odrive.find_any_async(odrive.default_usb_search_path, serial_number)
            logger.success("Device firmware update successful.")
        else:
            logger.success("Firmware upload successful.")
            logger.info("To complete the firmware update, set the DFU switch to \"RUN\" and power cycle the board.")


def _populate_sectors(sectors, sections: List[Tuple[int, bytes]]):
    """
    Checks for which on-device sectors there is data in the hex file and
    returns a (sector, data) tuple for each touched sector where data
    is a byte array of the same size as the sector.
    """
    for sector in sectors:
        sector_addr = sector['addr']
        sector_content = bytes([0xff]) * sector['len']

        # check if any segment from the hexfile overlaps with this sector
        touched = False
        for section_name, section_addr, section_content in sections:
            if section_addr + len(section_content) <= sector_addr:
                continue # section is completely before current sector
            if section_addr >= sector_addr + len(sector_content):
                continue # section is completely after current sector

            # prune start and end of section
            if section_addr < sector_addr:
                section_content = section_content[(sector_addr - section_addr):]
                section_addr = sector_addr
            if section_addr + len(section_content) > sector_addr + len(sector_content): # prune end
                section_content = section_content[:(sector_addr + len(sector_content) - section_addr)]
            
            # insert section data into sector
            sector_content = sector_content[:(section_addr - sector_addr)] + section_content + sector_content[(section_addr - sector_addr + len(section_content)):]
            touched = True

        if touched:
            yield (sector, sector_content)


def _get_first_mismatch_index(array1, array2):
    """
    Compares two arrays and returns the index of the
    first unequal item or None if both arrays are equal
    """
    if len(array1) != len(array2):
        raise Exception("arrays must be same size")
    for pos in range(len(array1)):
        if (array1[pos] != array2[pos]):
            return pos
    return None


def _show_deferred_message(delay: float, msg: str):
    async def msg_loop():
        while True:
            await asyncio.sleep(delay)
            print(msg)

    class MsgLoopCtx():
        def __init__(self):
            self._task = None
        async def __aenter__(self):
            self._task = asyncio.create_task(msg_loop())
            return self
        async def __aexit__(self, exc_type, exc_val, exc_tb):
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass # expected due to cancel() call above
    
    return MsgLoopCtx()
