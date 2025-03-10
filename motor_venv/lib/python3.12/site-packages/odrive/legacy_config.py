
import json
import os
import tempfile
from typing import List
import fibre.libfibre
import odrive
from odrive.utils import OperationAbortedException, yes_no_prompt

def _property_dict(device):
    def _impl(prefix, obj):
        for k in dir(obj):
            v = getattr(obj, k)
            if k.startswith('_') and k.endswith('_property'):
                yield '.'.join(prefix + [k[1:-9]]), v
            elif not k.startswith('_') and isinstance(v, fibre.libfibre.RemoteObject):
                yield from _impl(prefix + [k], v)
    return {k: v for k, v in list(_impl([], device))}

def _flatten(prefix: List[str], config: dict):
    for k, v in config.items():
        if isinstance(v, dict):
            yield from _flatten(prefix + [k], v)
        else:
            yield '.'.join(prefix + [k]), v

def restore_config(device, config: dict):
    """
    Parameters
    ----------
    device: The ODrive to write the config to.
    config: A dictionary of the form {path: value}
    """
    errors = []
    prop_dict = _property_dict(device)

    # flatten config dict for legacy compatibility
    config = {k: v for k, v in _flatten([], config)}

    for name, v in config.items():
        try:
            remote_attribute = prop_dict[name]
            if isinstance(v, str) and hasattr(type(remote_attribute), 'exchange') and type(remote_attribute).exchange._inputs[1][1] == 'object_ref':
                v = prop_dict[v]
            remote_attribute.exchange(v)
        except Exception as ex:
            errors.append("Could not restore {}: {}".format(name, str(ex)))

    return errors

def backup_config(device) -> dict:
    """
    Returns a dict of the form {path: value} containing all properties on the
    ODrive that have "config" in their path.

    Parameters
    ----------
    device: The device to read from
    """
    prop_dict = _property_dict(device)
    result = {}

    for k, prop in prop_dict.items():
        if ".config." in f".{k}.":
            val = prop.read()
            if isinstance(val, fibre.libfibre.RemoteObject):
                val = prop_dict[val]
                print("path:", val)
            result[k] = val

    return result


def get_temp_config_filename(device):
    serial_number = odrive.get_serial_number_str_sync(device)
    safe_serial_number = ''.join(filter(str.isalnum, serial_number))
    return os.path.join(tempfile.gettempdir(), 'odrive-config-{}.json'.format(safe_serial_number))

def backup_config_ui(device, filename, logger):
    """
    Exports the configuration of an ODrive to a JSON file.
    If no file name is provided, the file is placed into a
    temporary directory.
    """

    if filename is None:
        filename = get_temp_config_filename(device)

    logger.info("Saving configuration to {}...".format(filename))

    if os.path.exists(filename):
        if not yes_no_prompt("The file {} already exists. Do you want to override it?".format(filename), True):
            raise OperationAbortedException()

    data = backup_config(device)
    with open(filename, 'w') as file:
        json.dump(data, file, indent=2)
    logger.info("Configuration saved.")

def restore_config_ui(device, filename, logger):
    """
    Restores the configuration stored in a file 
    """

    if filename is None:
        filename = get_temp_config_filename(device)

    with open(filename) as file:
        data = json.load(file)

    logger.info("Restoring configuration from {}...".format(filename))
    errors = restore_config(device, data)

    for error in errors:
        logger.info(error)
    if errors:
        logger.warn("Some of the configuration could not be restored.")
    
    try:
        device.save_configuration()
    except fibre.libfibre.ObjectLostError:
        pass # Saving configuration makes the device reboot
    logger.info("Configuration restored.")
