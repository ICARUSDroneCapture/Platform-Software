
import odrive.database
from odrive.enums import EncoderId, AxisState, ComponentStatus, ProcedureResult, ODriveError, Rs485EncoderMode, Protocol
from odrive.hw_version import HwVersion
from odrive.rich_text import RichText, Color, Style
from enum import Enum
import enum
import time
from typing import NamedTuple, List, Set, Iterable, Dict, Optional
import re
import struct
import functools

class NetType(Enum):
    AB = enum.auto()
    DIGITAL = enum.auto()
    THREE_PHASE = enum.auto()
    DC = enum.auto()
    THERMISTOR = enum.auto()
    RS485 = enum.auto()
    CAN = enum.auto()
    ENC = enum.auto()

class EntityType(Enum):
    DEVICE = enum.auto()
    DEVICE_INVERTER = enum.auto()
    DEVICE_RS485_INTF = enum.auto()
    DEVICE_INC_ENC_INTF = enum.auto()
    DEVICE_ONBOARD_ENC = enum.auto()
    DEVICE_GPIO = enum.auto()
    AXIS = enum.auto()
    AXIS_MOTOR = enum.auto()
    AXIS_ENCODER = enum.auto()

class CalibrationStatus(Enum):
    OK = enum.auto()
    NEEDED = enum.auto()
    RECOMMENDED = enum.auto()
    UNKNOWN = enum.auto()

class IssueType(Enum):
    ERROR = enum.auto() # error level issues prevent the configuration from being written onto the ODrive(s)
    WARN = enum.auto() # warn level issues don't prevent the configuration from being applied but require user attention


class ConfigType():
    """
    Base class for config objects in the machine configuration. Implements
    similar functionality to typing.NamedTuple but with less restrictions.
    TODO: elaborate
    """
    def __init_subclass__(cls):
        cls._fields = [(k, t, getattr(cls, k)) for k, t in cls.__annotations__.items()]
    def __init__(self, **kwargs):
        for k, t, d in self.__class__._fields:
            if k in kwargs:
                setattr(self, k, kwargs[k])
            else:
                if isinstance(d, list):
                    d = [*d]
                setattr(self, k, d)
    def __repr__(self):
        return self.__class__.__name__ + "(" + ", ".join(k + "=" + repr(getattr(self, k)) for k, _, _ in self._fields) + ")"
    def __eq__(self, other):
        if self.__class__ != other.__class__:
            return False
        def eq(a, b):
            if isinstance(a, list) and isinstance(b, list):
                return len(a) == len(b) and all(eq(a[i], b[i]) for i in range(len(a)))
            return a == b
        return all(eq(getattr(self, k), getattr(other, k)) for k, _, _ in self.__class__._fields)
    def __hash__(self):
        def custom_hash(v):
            if isinstance(v, list):
                return custom_hash(tuple(elem for elem in v))
            return hash(v)
        return hash(tuple(custom_hash(getattr(self, k)) for k, _, _ in self.__class__._fields))

    def to_json(self):
        return _dump_named_tuple(self)

class EntityRef():
    def __init__(self, parent: 'EntityRef', name: str, key, entity_type: EntityType, net_type: Optional[NetType]):
        self.parent = parent
        self.name = name
        self.key = key
        self.entity_type = entity_type
        self.net_type = net_type

    def __eq__(self, other):
        return (isinstance(other, EntityRef) and
                self.parent == other.parent and
                self.entity_type == other.entity_type and
                self.key == other.key)
    def __hash__(self):
        return hash((self.parent, self.entity_type, self.key))
    def __repr__(self):
        return f'Entity[{self.name}]'

    def fullname(self):
        if self.name is None:
            return None
        else:
            parent_name = None if self.parent is None else self.parent.fullname()
            return ('' if parent_name is None else parent_name + '.') + self.name

    def get_children(self) -> Iterable['EntityRef']:
        return []


class DeviceRef(EntityRef):
    def __init__(self, machine_ref, dev_num, config: 'DeviceConfig'):
        EntityRef.__init__(self, machine_ref, f'devices[{dev_num}]', dev_num, EntityType.DEVICE, None)
        self.num = dev_num
        self.config = config

    def get_children(self):
        product_info = _db.get_product(self.config.board)
        for i in range(len(product_info['inverters'])):
            yield EntityRef(self, f'inverters[{i}]', i, EntityType.DEVICE_INVERTER, NetType.THREE_PHASE)
        for i in range(len(product_info['inc_enc'])):
            yield EntityRef(self, f'inc_enc[{i}]', i, EntityType.DEVICE_INC_ENC_INTF, NetType.AB)
        for i in range(product_info['onboard_encoders']):
            yield EntityRef(self, f'onboard_enc[{i}]', i, EntityType.DEVICE_ONBOARD_ENC, NetType.ENC)
        for i in range(len(product_info['rs485'])):
            yield EntityRef(self, f'rs485[{i}]', i, EntityType.DEVICE_RS485_INTF, NetType.RS485)
        for k, v in product_info['io'].items():
            yield EntityRef(self, f'io[{k}]', k, EntityType.DEVICE_GPIO, NetType.DIGITAL)
            if 't' in v:
                yield EntityRef(self, f'thermistor_input[{k}]', k, EntityType.DEVICE_GPIO, NetType.THERMISTOR)

class EncoderRef(EntityRef):
    def __init__(self, axis_ref, enc_num, config: 'EncoderConfig'):
        EntityRef.__init__(self, axis_ref, f'encoders[{enc_num}]', enc_num, EntityType.AXIS_ENCODER, NetType.ENC)
        self.num = enc_num
        self.config = config

    def get_children(self):
        # TODO: only yield the correct entities for this encoder
        yield EntityRef(self, f'ab', None, None, NetType.AB)
        yield EntityRef(self, f'z', None, None, NetType.DIGITAL)
        yield EntityRef(self, f'rs485', None, None, NetType.RS485)

class MotorRef(EntityRef):
    def __init__(self, axis_ref, motor_num, config: 'MotorConfig'):
        EntityRef.__init__(self, axis_ref, f'motors[{motor_num}]', motor_num, EntityType.AXIS_MOTOR, None)
        self.num = motor_num
        self.config = config

    def phases(self):
        return EntityRef(self, f'phases', None, None, NetType.THREE_PHASE)

    def get_children(self):
        yield self.phases()
        yield EntityRef(self, f'thermistor', None, None, NetType.THERMISTOR)

class AxisRef(EntityRef):
    def __init__(self, machine_ref, axis_num, config: 'AxisConfig'):
        EntityRef.__init__(self, machine_ref, f'axes[{axis_num}]', axis_num, EntityType.AXIS, None)
        self.num = axis_num
        self.config = config

    def motors(self):
        for i, config in enumerate(self.config.motors):
            yield MotorRef(self, i, config)

    def encoders(self):
        for i, config in enumerate(self.config.encoders):
            yield EncoderRef(self, i, config)

    def get_children(self):
        for motor_ref in self.motors():
            yield motor_ref
            yield from motor_ref.get_children()
        for encoder_ref in self.encoders():
            yield encoder_ref
            yield from encoder_ref.get_children()

class MachineRef(EntityRef):
    def __init__(self, config: 'MachineConfig'):
        EntityRef.__init__(self, None, None, None, None, None)
        self.config = config

    def devices(self):
        for i, dev_config in enumerate(self.config.devices):
            yield DeviceRef(self, i, dev_config)

    def axes(self):
        for i, axis_config in enumerate(self.config.axes):
            yield AxisRef(None, i, axis_config)

    def get_children(self):
        for dev_ref in self.devices():
            yield dev_ref
            yield from dev_ref.get_children()
        for axis_ref in self.axes():
            yield axis_ref
            yield from axis_ref.get_children()


_db = odrive.database.load()

_reboot_vars = [
    r'^inc_encoder[0-9]+.config.enabled$',
    r'^rs485_encoder_group0\.config\.mode$',
    r'^config\.enable_can_[a-z]$',
    r'^can\.condig\.baud_rate$',
    r'^axis[0-9]\.config\.load_encoder$',
    r'^axis[0-9]\.config\.commutation_encoder$',
    r'^axis[0-9]\.config\.can\.node_id$',
    r'^axis[0-9]\.config\.can\.is_extended$',
]

class ExpressionParseError(Exception):
    pass


def _parse_expr(val, unit, default):
    val = val.strip()
    if val == '':
        if default is None:
            return None
        val = default
    if unit != '':
        if not val.endswith(' ' + unit):
            raise ExpressionParseError(f"{val} is not a {unit}")
        val = val[:-len(unit)-1]
    try:
        return float(val)
    except ValueError:
        return ExpressionParseError(f"{val} is not a float")



class CalibrationTask():
    def __init__(self, name, func, status, issues):
        self.name = name
        self.func = func
        self.status = status
        self.issues = issues

    def run(self):
        self.func()

class IssueCollection():
    def __init__(self):
        self.issues = []

    def append(self, ref, message, level = IssueType.ERROR):
        """
        Appends an issue to the issue collection.

        `ref` defines which object in the user configuration the issue pertains
        to.
        """
        if isinstance(ref, EntityRef):
            self.issues.append((ref, message, level))
        elif isinstance(ref, list):
            for r in ref:
                self.issues.append((r, message, level))
        else:
            assert False, "bad issue reference: " + str(ref) + " - " + str(message)

    def get(self, ref: EntityRef):
        """
        Returns all issues pertaining to the specified configuration object.
        """
        for _ref, message, level in self.issues:
            if _ref == ref:
                yield message, level

    def get_for_type(self, ref_type: type):
        """
        Returns all issues pertaining to any object of the specified type.
        """
        for _ref, message, level in self.issues:
            if isinstance(_ref, ref_type):
                yield _ref, message, level

class AuxConfig(ConfigType):
    """
    Holds the configuration of a brake resistor in a user defined machine.
    """
    db_ref: str = ""
    resistance: str = ""

    @staticmethod
    def from_db(db_ref: str):
        return AuxConfig.from_json({'db_ref': db_ref})

    @staticmethod
    def from_json(json: dict) -> 'AuxConfig':
        """
        Loads a :class:`AuxConfig` object from a dictionary (usually loaded from JSON).
        """
        return _load_named_tuple(json, AuxConfig)
    
    def parse_resistance(self, unit=''): return _parse_expr(self.resistance, unit, None)

class EncoderConfig(ConfigType):
    """
    Holds the configuration of an encoder on an axis in a user defined machine.
    """
    db_ref: str = ""
    scale: str = "" # [scalar], default: 1.0

    @staticmethod
    def from_db(db_ref: str):
        return EncoderConfig.from_json({'db_ref': db_ref})

    @staticmethod
    def from_json(json: dict) -> 'EncoderConfig':
        """
        Loads a :class:`EncoderConfig` object from a dictionary (usually loaded from JSON).
        """
        return _load_named_tuple(json, EncoderConfig)

    def protocol(self):
        if self.db_ref == "":
            raise Exception("this encoder has no protocol property")
        encoder_info = _db.get_encoder(self.db_ref)
        return encoder_info['protocol']

class MotorConfig(ConfigType):
    """
    Holds the configuration of a motor on an axis in a user defined machine.
    """
    db_ref: str = ""
    scale: str = "" # [scalar], default: 1.0
    phase_resistance: str = "" # [Ohm]
    phase_inductance: str = "" # [H]
    use_thermistor: bool = False # default: False
    override: Dict = {}

    @staticmethod
    def from_db(db_ref: str, use_thermistor: bool = None, **kwargs):
        return MotorConfig.from_json({'db_ref': db_ref, 'use_thermistor': use_thermistor, **kwargs})

    @staticmethod
    def from_json(json: dict) -> 'MotorConfig':
        """
        Loads a :class:`MotorConfig` object from a dictionary (usually loaded from JSON).
        """
        return _load_named_tuple(json, MotorConfig)

    def parse_scale(self, unit=''): return _parse_expr(self.scale, unit, 1.0)
    def parse_phase_resistance(self, unit=''): return _parse_expr(self.phase_resistance, unit, None)
    def parse_phase_inductance(self, unit=''): return _parse_expr(self.phase_inductance, unit, None)

class AxisConfig(ConfigType):
    """
    Holds the configuration of an axis in a user defined machine.
    """
    motors: List[MotorConfig] = []
    encoders: List[EncoderConfig] = []
    calib_scan_vel: str = ""
    calib_scan_distance: str = ""
    calib_scan_range: str = ""
    calib_torque: str = ""
    pos_encoder: int = None
    vel_encoder: int = None
    commutation_encoder: int = None
    pos_index: str = ""
    pos_index_offset: str = ""
    can_protocol: str = ""
    can_node_id: str = ""

    @staticmethod
    def from_json(json: dict) -> 'AxisConfig':
        """
        Loads an :class:`AxisConfig` object from a dictionary (usually loaded from JSON).
        """
        return _load_named_tuple(json, AxisConfig)

    def parse_calib_scan_vel(self, unit=''): return _parse_expr(self.calib_scan_vel, unit, '0.275 rps')
    def parse_calib_scan_distance(self, unit=''): return _parse_expr(self.calib_scan_distance, unit, '1.1 turns')
    def parse_calib_scan_range(self, unit=''): return _parse_expr(self.calib_scan_range, unit, '0.02')
    def parse_calib_torque(self, unit=''): return _parse_expr(self.calib_torque, unit, None) # TODO: define default

    def add_motor(self, motor_config: MotorConfig):
        self.motors.append(motor_config)

    def add_encoder(self, encoder_type: str, cpr: int, use_for_pos: bool, use_for_vel: bool, use_for_commutation: bool):
        self.encoders.append(EncoderConfig({'type': encoder_type}))
        if use_for_pos:
            self.pos_encoder = len(self.encoders) - 1
        if use_for_vel:
            self.vel_encoder = len(self.encoders) - 1
        if use_for_commutation:
            self.commutation_encoder = len(self.encoders) - 1

class DeviceConfig(ConfigType):
    """
    Holds the configuration of an ODrive in a user defined machine.
    """
    board: HwVersion = None
    serial_number: str = ""
    shunt_conductances: List[float] = []
    aux: List[AuxConfig] = []
    fans: Dict[str, List[int]] = {}

    @staticmethod
    def from_json(json):
        return _load_named_tuple(json, DeviceConfig)

    def __init__(self, board: HwVersion, **kwargs):
        product_info = _db.get_product(board) # verify that this is a known product
        ConfigType.__init__(self, board=board, **kwargs)

JSON_FORMAT_VERSION = '0.1'

class FoundNoneError(Exception):
    pass

class FoundMultipleError(Exception):
    pass

class Net():
    @staticmethod
    def from_json(json: List[str], all_refs: Dict[str, EntityRef]):
        ports = set()
        types = set()
        for port_name in json:
            port = all_refs.get(port_name, None)
            if port is None:
                raise Exception("Unknown port: \"" + port_name + "\". Known ports are: " + str(list(all_refs.keys())))
            ports.add(port)
            types.add(port.net_type)

        assert len(types) == 1, types
        return Net(list(types)[0], ports)

    @staticmethod
    def union(net_type: NetType, *nets: Iterable['Net']):
        assert all((net.net_type == net_type for net in nets))
        if len(nets) == 0:
            return Net(net_type, set())
        return Net(net_type, set.union(*(net.ports for net in nets)))

    def __init__(self, net_type: NetType, ports: Set[EntityRef]):
        self.net_type = net_type
        self.ports = ports

    def __eq__(self, other):
        return isinstance(other, Net) and self.ports == other.ports
    def __hash__(self):
        return hash(self.ports)

    def __repr__(self):
        return 'Net(' + ', '.join(p.fullname() for p in self.ports) + ')'

    def to_json(self):
        return sorted(p.fullname() for p in self.ports)
    
    def get_all(self, entity_type: EntityType):
        """
        Returns all :class:`EntityRef`s that are connected to this net and have
        the specified type `entity_type`.
        """
        assert isinstance(entity_type, EntityType)
        def match(p):
            if p.entity_type == entity_type:
                return p
            elif p.parent is None:
                return None
            else:
                return match(p.parent)
        matches = [match(p) for p in self.ports]
        return [m for m in matches if not m is None]
    
    def get_single(self, entity_type: EntityType):
        """
        Like `get_all()`, but raises an exception if multiple or no matching
        entities were found.
        """
        ports = self.get_all(entity_type)
        if len(ports) == 0:
            raise FoundNoneError()
        elif len(ports) > 1:
            raise FoundMultipleError()
        return ports[0]

class Action():
    pass

class MotorCalibration(Action):
    def __init__(self, axis, motor_config):
        self.axis = axis
        self.motor_config = motor_config

    @property
    def name(self): return "Motor Calibration"

    @property
    def issues(self):
        if self.axis is None:
            return ["disconnected"]
        return []
    
    def run(self):
        assert len(self.issues) == 0
        time.sleep(1) # TODO: fix this
        # highly frequent (<100%) fail with current limit violation when run from test script, fixed with 1sec delay
        #  ^ does not seem to be an issue from `odrivetool`
        _run_state(self.axis, AxisState.MOTOR_CALIBRATION)
        print(f'measured phase_resistance: {self.axis.config.motor.phase_resistance}')
        print(f'measured phase_inductance: {self.axis.config.motor.phase_inductance}')

class EncoderCalibration(Action):
    def __init__(self, axis):
        self.axis = axis

    @property
    def name(self): return "Encoder Offset Calibration"

    @property
    def issues(self):
        if self.axis is None:
            return ["disconnected"]
        elif ComponentStatus(self.axis.commutation_mapper.status) == ComponentStatus.MISSING_INPUT:
            return ["Encoder disconnected or misconfigured."]
        return []

    def run(self):
        assert len(self.issues) == 0
        _run_state(self.axis, AxisState.ENCODER_OFFSET_CALIBRATION)

class ClosedLoopControl(Action):
    def __init__(self, axis, motor_config):
        self.axis = axis
        self.motor_config = motor_config
    
    @property
    def prerequisites(self):
        motor_calibrated = (self.motor_config.phase_resistance != "") and (self.motor_config.phase_inductance != "")
        yield MotorCalibration(self.axis, self.motor_config), CalibrationStatus.OK if motor_calibrated else CalibrationStatus.RECOMMENDED
        if ComponentStatus(self.axis.commutation_mapper.status) != ComponentStatus.NOMINAL:
            yield EncoderCalibration(self.axis), CalibrationStatus.NEEDED

class Axis():
    def __init__(self, handle, config):
        self.handle = handle
        self.config = config

    @property
    def closed_loop_control(self) -> Action:
        return ClosedLoopControl(self.handle, self.config.motors[0])


class MachineConfig(ConfigType):
    """
    Represents the configuration of a machine.

    This includes a list of axes, a list of ODrive devices and information about
    how the axes and devices are connected.
    """
    devices: List[DeviceConfig] = []
    axes: List[AxisConfig] = []
    nets: List[Net] = []

    @staticmethod
    def from_json(json: dict) -> 'MachineConfig':
        """
        Loads a :class:`MachineConfig` object from a dictionary (usually loaded from JSON).
        """
        if json.get('version', None) != JSON_FORMAT_VERSION:
            raise LoadConfigException("Unsupported config version " + str(json.get('version', None)))

        json = {**json}
        json.pop('version')
        nets = json.pop('nets') if 'nets' in json else []
        cfg = _load_named_tuple(json, MachineConfig)
        all_refs = cfg.get_entities_by_name()
        cfg.nets.extend([Net.from_json(net, all_refs) for net in nets])
        return cfg

    def __init__(self, **kwargs):
        nets = kwargs.pop('nets') if 'nets' in kwargs else []
        ConfigType.__init__(self, **kwargs)
        all_refs = self.get_entities_by_name()
        self.nets.extend([Net.from_json(net, all_refs) for net in nets])

    def to_json(self):
        json = _dump_named_tuple(self)
        json['version'] = JSON_FORMAT_VERSION
        return json

    def add_axis(self, axis_config: AxisConfig):
        """
        Adds an axis to the machine config.
        """
        self.axes.append(axis_config)
        return len(self.axes) - 1

    def add_device(self, dev_config: DeviceConfig):
        """
        Adds an ODrive device to the machine config.
        """
        self.devices.append(dev_config)
        return len(self.devices) - 1
    
    def merge(self, other: 'MachineConfig'):
        n_devices = len(self.devices)
        n_axes = len(self.axes)

        self.axes.extend(other.axes)
        self.devices.extend(other.devices)

        def shifted_name(ref: EncoderRef):
            if ref.entity_type == EntityType.DEVICE:
                my_name = f'devices[{ref.num + n_devices}]'
            elif ref.entity_type == EntityType.AXIS:
                my_name = f'axes[{ref.num + n_axes}]'
            else:
                my_name = ref.name
            parent_name = None if ref.parent is None else shifted_name(ref.parent)
            return my_name if (parent_name is None) else (parent_name + '.' + my_name)

        for net in other.nets:
            self.connect(*(shifted_name(ref) for ref in net.ports))

    def get_entities_by_name(self):
        machine_ref = MachineRef(self)
        return {ref.fullname(): ref for ref in machine_ref.get_children()}

    def connect(self, *refs):
        refs = set(refs)
        if len(refs) == 0:
            return

        all_ports = self.get_entities_by_name()
        ports = {all_ports[ref] for ref in refs}

        types = set(p.net_type for p in ports)
        if len(types) != 1:
            raise Exception(f"Cannot connect nets of different types: {types}")
        net_type = list(types)[0]
        
        for net in self.get_nets(net_type):
            intersection = net.ports & ports
            if len(intersection) > 0:
                net.ports.update(ports)
        
        self.nets.append(Net(net_type, ports))

#    def connect_phases(self, *refs):
#        assert all([isinstance(r, InverterRef) or isinstance(r, MotorRef) for r in refs])
#        self._three_phase_connections.append(refs)
#
#    def connect_abz(self, enc_ref, inc_enc_intf_ref, z_gpio_ref = None):
#        assert isinstance(enc_ref, EncoderRef)
#        assert isinstance(inc_enc_intf_ref, IncEncIntfRef)
#        assert z_gpio_ref is None or isinstance(z_gpio_ref, GpioRef)
#        # TODO: register connection for index signal
#        print("CONNECTING ", enc_ref.to_json(), inc_enc_intf_ref.to_json())
#        self._ab_connections.append([enc_ref, inc_enc_intf_ref])
#
#    def connect_rs485(self, enc_ref, rs485_intf_ref):
#        assert isinstance(enc_ref, EncoderRef)
#        assert isinstance(rs485_intf_ref, Rs485IntfRef)
#        self._rs485_connections.append([enc_ref, rs485_intf_ref])

    def get_nets(self, net_type: NetType):
        return [net for net in self.nets if net.net_type == net_type]

    def get_net(self, port: EntityRef):
        for net in self.nets:
            if any(p == port for p in net.ports):
                return net
        return Net(port.net_type, set())

    def get_status(self, odrives):
        """
        Returns various information about the machine configuration taking into
        account the list of currently connected ODrives and their state.

        Returns a tuple (odrv_list, output_configs, issues, axis_obj)
        where:

        odrv_list: A list of ODrive objects that need to be configured. Each
            entry corresponds to one device in this configuration object. Some
            entries can be None.
        output_configs: A list of multi-level dictionaries that hold all
            configuration settings for all ODrives, whether they are connected or
            not. The order an length of this list corresponds to `odrv_list`.
        issues: An `IssueCollection` containing all errors and warnings that
            were found.
        needs_reboot: A list of booleans indicating for each device if a reboot
            is required after applying the new configuration.
        axis_calib: A list of lists of CalibrationTask objects representing the
            available calibration tasks for this axis.
            Each list in axis_calib corresponds to an axis in this configuration.
        """

        output_configs = [{'config': {
                **{f'inverter{i}': {}
                for i in range(len(_db.get_product(d.board)['inverters']))},
                **{f'brake_resistor{i}': {}
                for i in range(len(_db.get_product(d.board)['aux_inverters']))}
            },
            **{
                f'axis{i}': {
                    'controller': {'config': {}}, 'config': {'motor': {}, 'can': {}},
                    'motor': {}
                    }
                for i in range(len(_db.get_product(d.board)['inverters']))
            }}
            for d in self.devices]
        issues = IssueCollection()
        axis_obj = [[] for _ in range(len(self.axes))]
        encoder_ids = [{} for _ in range(len(self.devices))]

        ref = MachineRef(self)

        # Associate devices in the configuration with connected devices
        odrv_list = [None for _ in range(len(self.devices))]
        odrives_by_serial_number = {odrv._serial_number: odrv for odrv in odrives}
        for dev_ref in ref.devices():
            dev_config = dev_ref.config
            if dev_config.serial_number is None:
                issues.append(dev_ref, 'Not associated with any serial number.')
            elif not dev_config.serial_number in odrives_by_serial_number:
                issues.append(dev_ref, 'Not connected.')
            else:
                dev = odrives_by_serial_number[dev_config.serial_number]
                board = dev._board
                if dev_config.board != board:
                    issues.append(dev_ref, 'Expected {} but found {}.'.format(dev_config.board, board))
                else:
                    odrv_list[dev_ref.num] = dev

        for odrv in odrives:
            if not odrv._serial_number in [c.serial_number for c in self.devices]:
                issues.append(ref, "Unused ODrive: " + odrv._serial_number, IssueType.WARN)

        # Configure RS485 encoders
        for net in self.get_nets(NetType.RS485):
            try:
                rs485_intf_ref = net.get_single(EntityType.DEVICE_RS485_INTF)
            except FoundNoneError:
                continue # not an error
            except FoundMultipleError as ex:
                issues.append(ex.refs, "Multiple ODrives are not allowed on the same RS485 bus.")
                continue

            enc_refs = net.get_all(EntityType.AXIS_ENCODER)
            odrv_output_config = output_configs[rs485_intf_ref.parent.num]

            for enc_ref in enc_refs:
                encoder_config = enc_ref.config
                enc_data = _db.get_encoder(encoder_config.db_ref)

                rs485_protocol = encoder_config.protocol()
                if not hasattr(Rs485EncoderMode, rs485_protocol):
                    issues.append(enc_ref, f"RS485 protocol {rs485_protocol} not supported.")
                    continue # ignore encoder

                # Each rs485_encoder_group is dedictated to one RS485 port on the ODrive.
                # One port can talk to multiple encoders (needs firmware change!).
                if f'rs485_encoder_group{rs485_intf_ref.key}' in odrv_output_config:
                    rs485_encoder_group_config = odrv_output_config[f'rs485_encoder_group{rs485_intf_ref.key}']
                else:
                    
                    rs485_encoder_group_config = {'config': {'mode': getattr(Rs485EncoderMode, rs485_protocol)}}
                    odrv_output_config[f'rs485_encoder_group{rs485_intf_ref.key}'] = rs485_encoder_group_config
                    
                encoder_ids[rs485_intf_ref.parent.num][enc_ref] = EncoderId.RS485_ENCODER0

        # Configure incremental encoders
        for net in self.get_nets(NetType.AB):
            try:
                enc_ref = net.get_single(EntityType.AXIS_ENCODER)
            except FoundNoneError:
                continue # not an error
            except FoundMultipleError as ex:
                issues.append(ex.refs, "Multiple incremental encoders cannot share the same A/B signals.")
                continue

            inc_enc_intf_refs = net.get_all(EntityType.DEVICE_INC_ENC_INTF)
            enc_data = _db.get_encoder(enc_ref.config.db_ref)

            for inc_enc_intf_ref in inc_enc_intf_refs:
                output_configs[inc_enc_intf_ref.parent.num]["inc_encoder{}".format(inc_enc_intf_ref.key)] = {
                    'config': {'enabled': True, 'cpr': enc_data['cpr']}
                }
                encoder_ids[inc_enc_intf_ref.parent.num][enc_ref] = [EncoderId.INC_ENCODER0, EncoderId.INC_ENCODER1][inc_enc_intf_ref.key]

        # Configure onboard encoders
        for net in self.get_nets(NetType.ENC):
            try:
                axis_enc_ref = net.get_single(EntityType.AXIS_ENCODER)
                onboard_enc_ref = net.get_single(EntityType.DEVICE_ONBOARD_ENC)
            except FoundNoneError:
                continue # not an error
            except FoundMultipleError as ex:
                issues.append(ex.refs, "Only 1:1 connections supported for onboard encoders.")
                continue
            encoder_ids[onboard_enc_ref.parent.num][axis_enc_ref] = [EncoderId.ONBOARD_ENCODER0, EncoderId.ONBOARD_ENCODER1][onboard_enc_ref.key]

        # Configure motors
        for net in self.get_nets(NetType.THREE_PHASE):
            try:
                inv_ref = net.get_single(EntityType.DEVICE_INVERTER)
            except FoundNoneError:
                continue # not connected to any inverter
            except FoundMultipleError:
                issues.append(axis_ref, "Phase bundling not implemented. Each motor must be connected to at most one inverter.")
                continue

            try:
                motor_ref = net.get_single(EntityType.AXIS_MOTOR)
            except FoundNoneError:
                continue # not connected to any inverter
            except FoundMultipleError:
                issues.append(axis_ref, "Each inverter must be connected to at most one motor.")
                continue

            motor_config = motor_ref.config
            motor_data = _db.get_motor(motor_config.db_ref)

            axis_output_config = output_configs[inv_ref.parent.key]['axis{}'.format(inv_ref.key)]

            if motor_config.scale != '':
                issues.append(motor_ref, "Support for motor scale other than 1.0 not implemented.")

            if motor_config.phase_resistance != "":
                axis_output_config['config']['motor']['phase_resistance'] = motor_config.parse_phase_resistance('Ohm')
            else:
                axis_output_config['config']['motor']['phase_resistance'] = motor_data['phase_resistance']

            if motor_config.phase_inductance != "":
                axis_output_config['config']['motor']['phase_inductance'] = motor_config.parse_phase_inductance('H')
            else:
                axis_output_config['config']['motor']['phase_inductance'] = motor_data['phase_inductance']

            if 'calibration_current' in motor_data:
                axis_output_config['config']['motor']['calibration_current'] = motor_data['calibration_current']

            # TODO: take into account user max current
            # Note: we multiply the motor current limit by two since it's given in "continuous max"
            inv_data = _db.get_product(self.devices[inv_ref.parent.key].board)['inverters'][inv_ref.key]
            axis_output_config['config']['motor']['current_soft_max'] = min(inv_data['max_current'], 2 * motor_data['max_current']) # TODO: inverter max current should be set as a separate config var
            axis_output_config['config']['motor']['current_hard_max'] = 1.5 * min(inv_data['max_current'], 2 * motor_data['max_current'])

            axis_output_config['config']['motor']['pole_pairs'] = motor_data['pole_pairs']
            axis_output_config['config']['motor']['torque_constant'] = motor_data['torque_constant']
            axis_output_config['config']['motor']['phase_resistance_valid'] = True
            axis_output_config['config']['motor']['phase_inductance_valid'] = True

        # Configure thermistors
        for net in self.get_nets(NetType.THERMISTOR):
            try:
                motor_ref = net.get_single(EntityType.AXIS_MOTOR)
            except FoundNoneError:
                continue # not connected to any inverter
            except FoundMultipleError:
                issues.append(motor_ref, "Each thermistor must be connected to at most one motor.")
                continue
            try:
                gpio_ref = net.get_single(EntityType.DEVICE_GPIO)
            except FoundNoneError:
                continue # not connected to any inverter
            except FoundMultipleError:
                issues.append(gpio_ref, "Each thermistor must be connected to at most one ODrive IO.")
                continue

            motor_config = motor_ref.config
            motor_data = {**_db.get_motor(motor_config.db_ref), **motor_config.override}
            # if motor_ref.num != gpio_ref.num:
            #     issues.append(motor_ref, f"The motor thermistor must either be disconnected or connected to the thermistor input that corresponds to the same ODrive and axis to which the motor is connected.")
            #     continue

            axis_output_config = output_configs[gpio_ref.parent.key]['axis{}'.format(0)]

            # axis_output_config = odrv_output_config['axis{}'.format(temp_in_num)]

            odrv_data = _db.get_product(gpio_ref.parent.config.board)

            temp_in_data = [temp_data for temp_data in  odrv_data['temp_in'] if temp_data['io'] == gpio_ref.key][0]

            axis_output_config['motor']['motor_thermistor'] = {
                'config': {
                    'r_ref': motor_data['thermistor_r25'],
                    'beta': motor_data['thermistor_beta'],
                    'temp_limit_lower': motor_data['min_temp'] if 'min_temp' in motor_data else motor_data['max_temp'] - 20,
                    'temp_limit_upper': motor_data['max_temp'],
                    'enabled': True # requires reboot (?)
                }
            }
            # TODO: set corresponding GPIO mode to 3 (or probably should be handled by firmware)

        # Configure shunt conductance
        for dev_num, dev_config in enumerate(self.devices):
            for inv_num in range(len(_db.get_product(dev_config.board)['inverters'])):
                if len(dev_config.shunt_conductances) > inv_num:
                    output_configs[dev_num]['config']['inverter{}'.format(inv_num)]['shunt_conductance'] = dev_config.shunt_conductances[inv_num]

        # Other configuration
        for dev_num, dev_config in enumerate(self.devices):
            output_config = output_configs[dev_num]

            # TODO: set vbus voltage trip level based on power supply setting
            # TODO: set dc_max_negative_current based on power supply setting
            for inv_num, aux in enumerate(dev_config.aux):
                if inv_num > len(_db.get_product(dev_config.board)['aux_inverters']):
                    continue # TODO: log issue if trying to config invalid aux inv
                if aux.resistance != "":
                    output_config['config'][f'brake_resistor{inv_num}']['resistance'] = aux.parse_resistance('Ohm')
                else:
                    output_config['config'][f'brake_resistor{inv_num}']['resistance'] = _db.get_brakeR(aux.db_ref)['resistance']

                # output_config['config'][f'brake_resistor{inv_num}']['enable'] = True # TODO: discuss desired default behavior

            output_config['config']['dc_max_negative_current'] = -1

            # fan config
            for id, limits in dev_config.fans.items():
                output_config['config'][id]= {
                    'lower': limits[0],
                    'upper': limits[1],
                    'enabled': True
                }

        # Configure axes
        for axis_ref in ref.axes():
            axis_config = axis_ref.config
            three_phase_net = Net.union(NetType.THREE_PHASE, *(self.get_net(motor_ref.phases()) for motor_ref in axis_ref.motors()))

            try:
                inv_ref = three_phase_net.get_single(EntityType.DEVICE_INVERTER)
            except FoundNoneError:
                issues.append(axis_ref, f"Not connected to any inverter.")
                continue
            except FoundMultipleError:
                issues.append(axis_ref, f"Connected to more than one inverters.")
                continue

            axis_output_config = output_configs[inv_ref.parent.num]['axis{}'.format(inv_ref.key)]

            if not axis_config.parse_calib_scan_vel('rps') is None:
                axis_output_config['config']['calib_scan_vel'] = axis_config.parse_calib_scan_vel('rps') * motor_data['pole_pairs'] * motor_config.parse_scale()
            
            if not axis_config.parse_calib_scan_distance('turns') is None:
                axis_output_config['config']['calib_scan_distance'] = axis_config.parse_calib_scan_distance('turns') * motor_data['pole_pairs'] * motor_config.parse_scale()
            
            if not axis_config.parse_calib_scan_range() is None:
                axis_output_config['config']['calib_range'] = axis_config.parse_calib_scan_range()
            
            # TODO: check if larger than current limit
            if not axis_config.parse_calib_torque('Nm') is None:
                axis_output_config['config']['calibration_lockin'] = {}
                axis_output_config['config']['calibration_lockin']['current'] = axis_config.parse_calib_torque('Nm') / motor_data['torque_constant']      

            if axis_config.pos_encoder is None:
                # TODO: use sensorless mode
                issues.append(axis_ref, "No position encoder specified")
            else:
                enc_ref = list(axis_ref.encoders())[axis_config.pos_encoder]
                enc_id = encoder_ids[inv_ref.parent.num].get(enc_ref, None)
                if enc_id is None:
                    issues.append(axis_ref, f"Load encoder of this axis must be connected to the same odrive as the motor ({output_configs[inv_ref.parent.num]['serial_number']})")
                else:
                    axis_output_config['config']['load_encoder'] = enc_id

            if axis_config.commutation_encoder is None:
                # TODO: use sensorless mode
                issues.append(axis_ref, "No commutation encoder specified")
            else:
                enc_ref = list(axis_ref.encoders())[axis_config.commutation_encoder]
                enc_id = encoder_ids[inv_ref.parent.num].get(enc_ref, None)
                if enc_id is None:
                    issues.append(axis_ref, f"Commutation encoder of this axis must be connected to the same odrive as the motor ({output_configs[inv_rev.parent.num]['serial_number']})")
                else:
                    axis_output_config['config']['commutation_encoder'] = enc_id

            if axis_config.vel_encoder is None:
                # TODO: use sensorless mode
                issues.append(axis_ref, "No commutation encoder specified")
            elif axis_config.vel_encoder == axis_config.commutation_encoder:
                axis_output_config['controller']['config']['use_commutation_vel'] = False
            elif axis_config.vel_encoder == axis_config.pos_encoder:
                axis_output_config['controller']['config']['use_commutation_vel'] = True
            else:
                issues.append(axis_ref, "The velocity encoder must be the same as either the position encoder or the commutation encoder.")

            if axis_config.can_protocol == "":
                pass
            elif axis_config.can_protocol == "simple":
                output_configs[inv_ref.parent.num]['can'] = {"config": {"protocol": Protocol.SIMPLE}}
                axis_output_config['config']['can']['node_id'] = int(axis_config.can_node_id)
            else:
                issues.append(axis_ref, f"Unknown CAN protocol {axis_config.can_protocol}.")

            #if enc_id in [EncoderId.INC_ENCODER0, EncoderId.INC_ENCODER1]:
            axis = None if odrv_list[inv_ref.parent.num] is None else getattr(odrv_list[inv_ref.parent.num], 'axis{}'.format(inv_ref.key))
            axis_obj[axis_ref.num] = Axis(axis, axis_config)

        def strip_empty_fields(d):
            result = {}
            for k, v in d.items():
                if isinstance(v, dict):
                    v = strip_empty_fields(v)
                    if v == {}:
                        continue
                result[k] = v
            return result

        output_configs = [strip_empty_fields(cfg) for cfg in output_configs]

        def compare(path, obj, config):
            all_equal = True
            reboot_required = False

            for k, v in config.items():
                if isinstance(v, dict):
                    equal, sub_reboot = compare(path + [k], getattr(obj, k), v)
                    reboot_required = reboot_required or sub_reboot
                elif isinstance(v, float):
                    # TODO: this comparison is a bit fragile (shouldn't compare floats like this)
                    equal = getattr(obj, k) == struct.unpack("f", struct.pack("f", v))[0]
                elif isinstance(v, enum.Enum):
                    equal = v.value == getattr(obj, k)
                else:
                    equal = getattr(obj, k) == v

                all_equal = all_equal and equal
                if not equal:
                    name = '.'.join(path + [k])
                    #print("changed: ", name)
                    if any(re.match(r, name) for r in _reboot_vars):
                        reboot_required = True
            return all_equal, reboot_required

        needs_reboot = [False] * len(odrv_list)
        for dev_ref in ref.devices():
            odrv = odrv_list[dev_ref.num]
            if not odrv is None:
                all_equal, needs_reboot[dev_ref.num] = compare([], odrv, output_configs[dev_ref.num])
                if not all_equal:
                    issues.append(dev_ref, "Configuration needs to be committed to ODrive", IssueType.WARN)
                needs_reboot[dev_ref.num] = needs_reboot[dev_ref.num] or odrv.reboot_required
                if needs_reboot[dev_ref.num]:
                    issues.append(dev_ref, "Reboot required", IssueType.WARN)

        return odrv_list, output_configs, issues, needs_reboot, axis_obj
#

    def format_status(self, odrives) -> RichText:
        """
        Returns a status summary of the configuration in a human readable format.
        This includes warnings about any issues with the configuration, calibration
        status and more.
        """

        odrv_list, output_configs, issues, needs_reboot, axis_obj = self.get_status(odrives)

        lines = []

        check_sign = "\u2705"
        info_sign = "\U0001F4A1"
        warning_sign = "\u26A0\uFE0F "
        error_sign = "\u274C"
        question_sign = "  " # TODO

        sign = {
            IssueType.WARN: warning_sign,
            IssueType.ERROR: error_sign
        }

        style = {
            IssueType.WARN: (Color.YELLOW, Color.DEFAULT, Style.BOLD),
            IssueType.ERROR: (Color.RED, Color.DEFAULT, Style.BOLD)
        }

        ref = MachineRef(self)

        for message, level in issues.get(ref):
            lines.append(sign[level] + " " + RichText(message, *style[level]))

        for dev_ref in ref.devices():
            dev_config = dev_ref.config
            if not dev_config.serial_number is None:
                name = "ODrive with serial number " + str(dev_config.serial_number)
            else:
                name = "ODrive {} (unspecified serial number)".format(dev_ref.num)
            lines.append(name)
            for message, level in issues.get(dev_ref):
                lines.append("  " + sign[level] + " " + RichText(message, *style[level]))

        for axis_ref in ref.axes():
            axis_config = axis_ref.config
            lines.append("Axis " + str(axis_ref.num))
            for message, level in issues.get(axis_ref):
                lines.append("  " + sign[level] + " " + RichText(message, *style[level]))
            for motor_ref in axis_ref.motors():
                for message, level in issues.get(motor_ref):
                    lines.append("  " + sign[level] + " Motor " + str(motor_ref.num) + ": " + RichText(message, *style[level]))
            for enc_ref in axis_ref.encoders():
                for message, level in issues.get(enc_ref):
                    lines.append("  " + sign[level] + " Encoder " + str(enc_ref.num) + ": " + RichText(message, *style[level]))


            # TODO: the calibration status is not really meaningful if there is
            # uncommitted configuration. needs_reboot is not the correct thing
            # to check but it's close enough for now
            if any(needs_reboot):
                continue

            for calib, calib_status in axis_obj[axis_ref.num].closed_loop_control.prerequisites:
                if calib_status == CalibrationStatus.OK:
                    lines.append("  " + check_sign + " " + RichText(str(calib.name) + " ok", Color.GREEN))
                elif calib_status == CalibrationStatus.RECOMMENDED:
                    lines.append("  " + info_sign + " " + RichText(str(calib.name) + " recommended"))
                elif calib_status == CalibrationStatus.NEEDED:
                    lines.append("  " + warning_sign + " " + RichText(str(calib.name) + " needed", style=Style.BOLD))
                elif calib_status == CalibrationStatus.UNKNOWN:
                    lines.append("  " + question_sign + " " + RichText(str(calib.name) + ": unknown status", Color.GRAY))
                else:
                    assert(False)
                for issue in calib.issues:
                    lines.append("    " + RichText(str(calib.name) + ": " + issue, Color.RED))

        return RichText("\n").join(lines)


    def apply(self, odrives):
        """
        Commits the configuration to the odrives. A reboot may be needed after
        this.

        If there a are problems with the configuration this function throws an
        exception and does not change anything on any ODrive.
        In this case show_status() can be used to get more detailed error
        information.

        Returns a list of devices that need a reboot before the configuration
        takes effect.
        """

        odrv_list, output_configs, issues, needs_reboot, axis_obj = self.get_status(odrives)

        if any([m for _, m, level in issues.issues if level == IssueType.ERROR]):
            print([m for _, m, level in issues.issues if level == IssueType.ERROR])
            raise Exception("There are problems with this configuration. No changes were applied to the ODrive(s).")

        def _apply(obj, config):
            for k, v in config.items():
                if isinstance(v, dict):
                    _apply(getattr(obj, k), v)
                else:
                    setattr(obj, k, v)

        for odrv_num, odrv in enumerate(odrv_list):
            if not odrv is None:
                if needs_reboot[odrv_num]:
                    odrv.reboot_required = True
                _apply(odrv, output_configs[odrv_num])

        return [odrv_list[num] for num, r in enumerate(needs_reboot) if r]

    def calibrate(self, odrives, include_optional = True):
        """
        Runs the calibration tasks for this machine configuration based on the
        current state of the ODrives. This can include a reboot of one or more
        ODrives.
        """

        odrv_list, output_configs, issues, needs_reboot, axis_obj = self.get_status(odrives)

        if any(needs_reboot):
            raise Exception("Some devices need to be rebooted for the configuration to take effect.")

        for axis_num, axis in enumerate(axis_obj):
            for calib, calib_status in axis.closed_loop_control.prerequisites:
                if (calib_status == CalibrationStatus.RECOMMENDED and include_optional) or calib_status == CalibrationStatus.NEEDED:
                    issues = list(calib.issues)
                    if len(issues):
                        raise Exception(f"Can't run {calib.name}: {issues}")
                    print(f"Running {calib.name} on axis {axis_num}...")
                    calib.run()
        print("Done!")

class LoadConfigException(Exception):
    pass

def _load_py_obj(data, py_type):
    if py_type == str:
        if isinstance(data, str):
            return data
        else:
            raise LoadConfigException(f"expected str but got {data}")

    elif py_type == bool:
        if isinstance(data, bool):
            return data
        else:
            raise LoadConfigException(f"expected bool but got {data}")

    elif py_type == int:
        if isinstance(data, int):
            return data
        else:
            raise LoadConfigException(f"expected int but got {data}")

    elif (hasattr(py_type, '_name') and py_type._name == 'List') or (hasattr(py_type, '__name__') and py_type.__name__ == 'List'): # __name__ check is for Python 3.6 compatibility
        elem_type = py_type.__args__[0]
        if isinstance(data, list):
            return [_load_py_obj(v, elem_type) for v in data if not v is None]
        else:
            raise LoadConfigException(f"expected list but got {data}")

    elif hasattr(py_type, 'from_json'):
        return py_type.from_json(data)

    elif (hasattr(py_type, '_name') and py_type._name == 'Dict') or (hasattr(py_type, '__name__') and py_type.__name__ == 'Dict'): # __name__ check is for Python 3.6 compatibility
        elem_type = py_type.__args__[1]
        if isinstance(data, dict):
            return {
                i: _load_py_obj(v, elem_type) 
                for i,v in data.items() if not v is None}
        else:
            raise LoadConfigException(f"expected dict but got {data}")
        
    elif hasattr(py_type, '__name__') and py_type.__name__ in ('T', 'KT', 'VT'):
        return data
    
    else:
        # This is not the fault of the user config but of our own code
        raise Exception(f"Don't know how to decode JSON to {py_type.__name__}")

def _load_named_tuple(data, py_type, **kwargs):
    assert isinstance(data, dict)

    unsupported_keys = set(data.keys()) - (set(py_type.__annotations__.keys()) - set(kwargs.keys()))
    if len(unsupported_keys) > 0:
        raise Exception(f"Unexpected keys for {py_type.__name__}: {unsupported_keys}")

    init_dict = {}
    for field_name, field_type, field_default in py_type._fields:
        field_val = data.get(field_name, None)
        if field_val is None:
            continue
        init_dict[field_name] = _load_py_obj(field_val, field_type)

    return py_type(**init_dict, **kwargs)


def _is_default(val, default):
    if val == default:
        return True
    elif type(val) == str:
        return val.strip() == default
    return False

def _dump_py_obj(py_obj):
    if py_obj is None:
        return None
    elif type(py_obj) == str:
        return None if py_obj.strip() == "" else py_obj.strip()
    elif type(py_obj) == int:
        return py_obj
    elif type(py_obj) == float:
        return py_obj
    elif type(py_obj) == bool:
        return py_obj
    elif type(py_obj) == list:
        return None if (len(py_obj) == 0) else [_dump_py_obj(o) for o in py_obj]
    elif type(py_obj) == dict:
        return None if (len(py_obj) == 0) else {i:_dump_py_obj(o) for i,o in py_obj.items()}
    elif hasattr(py_obj, 'to_json'):
        return py_obj.to_json()
    else:
        raise Exception(f"Don't know how to dump object {py_obj}")

def _dump_named_tuple(py_obj):
    data = {}
    for k, _, d in py_obj.__class__._fields:
        subdata = getattr(py_obj, k)
        if _is_default(subdata, d):
            continue # skip
        data[k] = _dump_py_obj(subdata)
    return data


def _run_state(axis, state):
    axis.requested_state = state
    while AxisState(axis.requested_state) == state:
        time.sleep(0.1)
    while ProcedureResult(axis.procedure_result) == ProcedureResult.BUSY:
        time.sleep(0.1)

    result = ProcedureResult(axis.procedure_result)
    if result == ProcedureResult.DISARMED:
        raise Exception("Device failed with {}".format(repr(ODriveError(axis.disarm_reason))))
    elif result != ProcedureResult.SUCCESS:
        raise Exception("Device returned {}".format(repr(ProcedureResult(result))))
