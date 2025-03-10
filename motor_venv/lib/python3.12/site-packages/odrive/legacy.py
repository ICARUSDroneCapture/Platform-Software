"""
Functions to support legacy firmware
"""

from odrive.rich_text import RichText, Color, Style
import odrive.utils

def format_errors(odrv, clear=False):
    """
    Returns a summary of the error status of the device as RichText.
    """
    lines = []

    STYLE_GOOD = (Color.GREEN, Color.DEFAULT, Style.BOLD)
    STYLE_WARN = (Color.YELLOW, Color.DEFAULT, Style.BOLD)
    STYLE_BAD = (Color.RED, Color.DEFAULT, Style.BOLD)

    axes = [(name, getattr(odrv, name)) for name in dir(odrv) if name.startswith('axis')]
    axes.sort()

    def decode_flags(val, enum_type):
        errorcodes = {v.value: f"{enum_type.__name__}.{v.name}" for v in enum_type}
        if val == 0:
            return [RichText("no error", *STYLE_GOOD)]
        else:
            return [RichText("Error(s):", *STYLE_BAD)] + [
                RichText(errorcodes.get((1 << bit), 'UNKNOWN ERROR: 0x{:08X}'.format(1 << bit)), *STYLE_BAD)
                for bit in range(64) if val & (1 << bit) != 0]

    def decode_enum(val, enum_type):
        errorcodes = {v.value: f"{enum_type.__name__}.{v.name}" for v in enum_type}
        return [RichText(errorcodes.get(val, 'Unknown value: ' + str(val)), *(STYLE_GOOD if (val == 0) else STYLE_BAD))]

    def decode_drv_fault(axis, val):
        if val == 0:
            return [RichText("none", *STYLE_GOOD)]
        elif not hasattr(axis, '_metadata'):
            return [RichText("metadata not loaded", *STYLE_WARN)]
        else:
            return [RichText(odrive.utils.decode_drv_faults(axis._metadata['drv'], val), *STYLE_BAD)]

    def dump_item(indent, name, obj, path, decoder):
        prefix = indent + name.strip('0123456789') + ": "
        for elem in path.split('.'):
            if not hasattr(obj, elem):
                return [prefix + RichText("not found", *STYLE_WARN)]
            obj = getattr(obj, elem)

        lines = decoder(obj)
        lines = [indent + name + ": " + lines[0]] + [
            indent + "  " + line
            for line in lines[1:]
        ]
        return lines

    lines += dump_item("", "system", odrv, 'error', lambda x: decode_flags(x, odrive.enums.LegacyODriveError))

    for name, axis in axes:
        lines.append(name)
        lines += dump_item("  ", 'axis', axis, 'error', lambda x: decode_flags(x, odrive.enums.AxisError))
        lines += dump_item("  ", 'motor', axis, 'motor.error', lambda x: decode_flags(x, odrive.enums.MotorError))
        lines += dump_item("  ", 'DRV fault', axis, 'last_drv_fault', lambda x: decode_drv_fault(axis, x))
        lines += dump_item("  ", 'sensorless_estimator', axis, 'sensorless_estimator.error', lambda x: decode_flags(x, odrive.enums.SensorlessEstimatorError))
        lines += dump_item("  ", 'encoder', axis, 'encoder.error', lambda x: decode_flags(x, odrive.enums.EncoderError))
        lines += dump_item("  ", 'controller', axis, 'controller.error', lambda x: decode_flags(x, odrive.enums.ControllerError))

    if clear:
        odrv.clear_errors()

    if clear:
        odrv.clear_errors()

    return RichText('\n').join(lines)
