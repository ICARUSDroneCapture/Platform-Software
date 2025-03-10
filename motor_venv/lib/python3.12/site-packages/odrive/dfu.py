import aiohttp
import asyncio
import os
from typing import Optional

import fibre
import odrive
import odrive.firmware
from odrive.release_api import VersionRelationship, format_version
from .hw_version import HwVersion
from .libodrive import Interface, LibODrive, Device, Firmware

class DfuError(Exception):
    pass

async def enter_dfu_mode(device, discoverer: Interface) -> Device:
    """
    Puts the specified device into (new) DFU mode.
    """
    serial_number = "{:08X}".format(device.serial_number)
    print("Putting device {:08X} into DFU mode...".format(device.serial_number))
    try:
        result = device.enter_dfu_mode2()
    except fibre.ObjectLostError:
        result = True # this is expected because the device reboots
    if not result:
        raise DfuError("Failed to enter DFU mode.")

    device = await discoverer.wait_for(serial_number, is_bootloader=True)
    await device.connect_bootloader()
    return device

async def write_firmware(device: Device, fw: Firmware, erase_all: bool):
    def print_progress(new_action_group: bool, action_string: str, action_index: int, n_actions: int):
        if new_action_group and action_index != 0:
            print()
        print(f"DFU: {action_string}    ", end='\r')
    try:
        await device.run_installation(fw, erase_all, print_progress)
    finally:
        print()

async def await_first(tasks):
    """
    Awaits the first of several tasks or futures and cancels the others.
    """
    # TODO: is there any standard way to do this in one line?
    done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    for t in pending:
        t.cancel()
    for t in pending:
        try:
            await t
        except asyncio.CancelledError:
            pass
    return (await asyncio.gather(*done))[0]

async def get_firmware(board: HwVersion, current_build_id_short: Optional[str], channel: Optional[str], version: Optional[str], logger, interactive: bool, release_type: str = 'firmware'):
    async with aiohttp.ClientSession() as session:
        if channel:
            logger.info(f"Checking online for latest {board.display_name} {release_type} on channel {channel}...")
        else:
            logger.info(f"Checking online for {board.display_name} {release_type} version {format_version(version)}...")
        api_client = odrive.api_client.ApiClient(session)
        release_api = odrive.release_api.ReleaseApi(api_client)

        firmware_index = await release_api.get_index(release_type)

        # If we're fetching normal firmware, use whatever file the release
        # server returns as preferred file URL.
        # If we're fetching the bootloader, need to select between multiple
        # files on the release server.
        file = 'bootloader_installer.elf' if release_type == 'bootloader' else None

        try:
            if channel:
                manifest = firmware_index.get_latest(channel, app='default', board=board, file=file)
            else:
                manifest = firmware_index.get_version(version, app='default', board=board, file=file)
        except odrive.release_api.ChannelNotFoundError as ex:
            raise DfuError(ex)
        except odrive.release_api.FirmwareNotFoundError:
            raise DfuError(f"No {release_type} found matching the specified criteria.")

        if interactive:
            version_relationship = firmware_index.compare(current_build_id_short, manifest['commit_hash'], channel, app='default', board=board)
            prompt = {
                VersionRelationship.UNKNOWN: "Found compatible {release_type} ({to_version}). Install now?",
                VersionRelationship.EQUAL: "Your current {release_type} ({to_version}) is up to date. Do you want to reinstall this version?",
                VersionRelationship.UPGRADE: "Found new {release_type} ({from_hash} => {to_version}). Install now?",
                VersionRelationship.DOWNGRADE: "Found older {release_type} ({from_hash} => {to_version}). Install now?",
            }[version_relationship]
        
            if not odrive.utils.yes_no_prompt(prompt.format(release_type=release_type, from_hash=current_build_id_short, to_version=format_version(manifest['commit_hash'])), True):
                raise odrive.utils.OperationAbortedException()

        logger.info(f"Downloading {release_type}...")
        return odrive.firmware.FirmwareFile.from_file(await release_api.load(manifest))

async def run_dfu(libodrive: LibODrive, dfu_discoverer: Interface, serial_number: Optional[str], path: Optional[str], channel: Optional[str], version: Optional[str], erase_all: bool, logger, interactive: bool = True):
    """
    See dfu_ui for description.
    """
    assert sum([bool(path), bool(channel), bool(version)]) == 1

    logger.info("Waiting for ODrive...")

    # Wait for device either in DFU mode or in normal mode, whichever is
    # found first.
    device = await await_first([
        asyncio.create_task(odrive.find_any_async(serial_number=serial_number)),
        asyncio.create_task(dfu_discoverer.wait_for(serial_number=serial_number, is_bootloader=True))
    ])

    found_in_dfu = isinstance(device, Device)

    if not found_in_dfu:
        bootloader_version = device.bootloader_version if hasattr(device, "bootloader_version") else 0
        if bootloader_version == 0:
            raise DfuError(
                "New DFU system not installed on device {:08X}.\n"
                "Please follow instructions for one-time setup here:\n"
                "https://docs.odriverobotics.com/v/latest/guides/new-dfu.html\n"
                "or use the legacy DFU system (odrivetool legacy-dfu)."
                .format(device.serial_number)
            )

        # Note: we don't do ahead of time compatibility check until the
        # bootloader version has stabilized. Just-in-time checking is
        # handled in libodrive (when bootloader is already started).

    if found_in_dfu:
        await device.connect_bootloader()

    if path:
        assert os.path.isfile(path)
        assert channel is None
        file = odrive.firmware.FirmwareFile.from_file(path)
    else:
        board: HwVersion = device.hw_version if isinstance(device, Device) else device._board
        build_id_short: Optional[str] = "{:08x}".format(device.commit_hash) if hasattr(device, 'commit_hash') else None
        file = await get_firmware(board, build_id_short, channel, version, logger, interactive)

    with libodrive.open_firmware(file.as_buffer()) as firmware:
        print("loaded firmware: ")
        print("  Version: " + str(".".join(str(n) for n in firmware.fw_version)))
        print("  Build ID: " + "".join(f"{b:02x}" for b in firmware.build))
        print("  Hardware: " + firmware.hw_version.display_name)

        if not isinstance(device, Device):
            device = await enter_dfu_mode(device, dfu_discoverer)

        assert device.is_bootloader
        await write_firmware(device, firmware, erase_all)

    logger.info("Waiting for the device to reappear...")
    device = await odrive.find_any_async(odrive.default_usb_search_path, serial_number)
    logger.success("Device firmware update successful.")


async def dfu_ui(serial_number: Optional[str], path: Optional[str], channel: Optional[str], version: Optional[str], erase_all: bool, logger, interactive: bool = True):
    """
    Runs the complete interactive DFU process:

    1. Wait for device in either DFU mode or normal mode. If `serial_number` is
       None, the first discovered device is selected, otherwise only the
       specified device is accepted.

    2. If `path` is None, check for the latest or specified firmware, present it
       to the user and ask whether to continue. Otherwise don't ask and always
       continue.

    3. If the device is in normal mode, put it into DFU mode.

    4. Erase, write and verify flash memory.

    5. Exit DFU mode.

    Parameters
    ----------
    path: Path to a .elf path or None to check online.
    channel: Channel on which to check for firmware (master, devel, ...)
    version: Exact firmware version
    """
    libodrive = LibODrive(loop=asyncio.get_running_loop())

    try:
        with libodrive.start_usb_discovery() as dfu_discoverer:
            libodrive._worker_thread.start()
            try:
                await run_dfu(libodrive, dfu_discoverer, serial_number, path, channel, version, erase_all, logger, interactive)
            finally:
                libodrive.stop_thread()
    finally:
        libodrive.deinit()
