import asyncio
import os
import sys
import platform
import threading
import fibre
import odrive
import odrive.enums
import odrive.config
import odrive.legacy_config
import odrive.dfu
import odrive.utils

def print_banner():
    print("Website: https://odriverobotics.com/")
    print("Docs: https://docs.odriverobotics.com/")
    print("Forums: https://discourse.odriverobotics.com/")
    print("Discord: https://discord.gg/k3ZZ3mS")
    print("Github: https://github.com/odriverobotics/ODrive/")

    print()
    print('Please connect your ODrive.')
    print('You can also type help() or quit().')

def print_help(args, have_devices):
    print('')
    if have_devices:
        print('Connect your ODrive to {} and power it up.'.format(args.path))
        print('After that, the following message should appear:')
        print('  "Connected to ODrive [serial number] as odrv0"')
        print('')
        print('Once the ODrive is connected, type "odrv0." and press <tab>')
    else:
        print('Type "odrv0." and press <tab>')
    print('This will present you with all the properties that you can reference')
    print('')
    print('For example: "odrv0.axis0.encoder.pos_estimate"')
    print('will print the current encoder position on axis 0')
    print('and "odrv0.axis0.controller.input_pos = 0.5"')
    print('will send axis 0 to 0.5 turns')
    print('')


interactive_variables = {}

discovered_devices = []

def benchmark(odrv):
    import time

    async def measure_async():
        start = time.monotonic()
        futures = [odrv.vbus_voltage for i in range(1000)]
#        data = [await f for f in futures]
#        print("took " + str(time.monotonic() - start) + " seconds. Average is " + str(sum(data) / len(data)))

    fibre.libfibre.libfibre.loop.call_soon_threadsafe(lambda: asyncio.ensure_future(measure_async()))

class ShellVariables():
    odrive = odrive
    devices = []
    config = odrive.config.MachineConfig()

    @staticmethod
    def apply():
        ShellVariables.config.apply(ShellVariables.devices)

    @staticmethod
    def status():
        odrive.rich_text.print_rich_text(ShellVariables.config.format_status(ShellVariables.devices))

    @staticmethod
    def calibrate():
        ShellVariables.config.calibrate(ShellVariables.devices)

def _import_from(source):
    return {
        k: getattr(source, k)
        for k in dir(source)
        if not k.startswith("_")
    }

def _wrap_async(func):
    def wrapper(*args, **kwargs):
        return asyncio.run(func(*args, **kwargs))
    return wrapper

def launch_shell(args, logger):
    """
    Launches an interactive python or IPython command line
    interface.
    As ODrives are connected they are made available as
    "odrv0", "odrv1", ...
    """

    interactive_variables = {
        'start_liveplotter': odrive.utils.start_liveplotter,
        'dump_errors': odrive.utils.dump_errors,
        'benchmark': benchmark,
        'oscilloscope_dump': odrive.utils.oscilloscope_dump,
        'dump_interrupts': odrive.utils.dump_interrupts,
        'dump_threads': odrive.utils.dump_threads,
        'dump_dma': odrive.utils.dump_dma,
        'dump_timing': odrive.utils.dump_timing,
        'ram_osci_config': _wrap_async(odrive.utils.ram_osci_config),
        'ram_osci_trigger': odrive.utils.ram_osci_trigger,
        'ram_osci_download': _wrap_async(odrive.utils.ram_osci_download),
        'ram_osci_run': _wrap_async(odrive.utils.ram_osci_run),
        'BulkCapture': odrive.utils.BulkCapture,
        'step_and_plot': odrive.utils.step_and_plot,
        'calculate_thermistor_coeffs': odrive.utils.calculate_thermistor_coeffs,
        'set_motor_thermistor_coeffs': odrive.utils.set_motor_thermistor_coeffs,
        'backup_config': odrive.legacy_config.backup_config,
        'restore_config': odrive.legacy_config.restore_config,
    }

    # Import a bunch of variables and functions from various sources
    interactive_variables.update(_import_from(ShellVariables))
    interactive_variables.update(_import_from(odrive.enums))
    interactive_variables.update(_import_from(odrive.config))

    private_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'odrive_private')
    if os.path.isfile(os.path.join(private_path, '__init__.py')):
        print("loading private plugins...")
        sys.path.insert(0, private_path)
        import odrive_private
        odrive_private.load_odrivetool_plugins(interactive_variables)

    async def mount(obj):
        serial_number_str = await odrive.utils.get_serial_number_str(obj)
        if ((not args.serial_number is None) and (serial_number_str != args.serial_number)):
            return None # reject this object
        if hasattr(obj, '_otp_valid_property') and not await obj._otp_valid_property.read():
            logger.warn("Device {}: Not a genuine ODrive! Some features may not work as expected.".format(serial_number_str))
            return ("device " + serial_number_str, serial_number_str, "dev")
        await odrive.utils.attach_metadata(obj)
        fw_version_str = '???' if obj._fw_version is None else 'v{}.{}.{}'.format(*obj._fw_version)
        return (f"{obj._board.display_name if not obj._board is None else 'device'} {serial_number_str} (firmware {fw_version_str})", serial_number_str, "odrv")

    fibre.launch_shell(args, mount,
                       interactive_variables,
                       ShellVariables.devices,
                       print_banner, print_help,
                       logger)
