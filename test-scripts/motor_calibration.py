import odrive
import time
import math
import csv
import sched

# ANSI escape codes for colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

# Define the first (and for now only) as our main odrv object
print("Connecting to ODrive...")
# odrv = odrv0
odrv = odrive.find_any()

print("Clearing ODrive errors...")
if odrv:
    odrv.clear_errors()
    print(f"{GREEN}✅ ODrive errors cleared.{RESET}")
else:
    print(f"{RED}❌ Error: ODrive not found!{RESET}")
    exit()

## Control Modes

# ControlMode.VOLTAGE_CONTROL     -> 0
# ControlMode.TORQUE_CONTROL      -> 1
# ControlMode.VELOCITY_CONTROL    -> 2
# ControlMode.POSITION_CONTROL    -> 3

# (These numbers are not a typo, this is what the docs say...)
# MotorType.HIGH_CURRENT        -> 0
# MotorType.GIMBAL              -> 2
# MotorType.ACIM                -> 3

# InputMode.INACTIVE            -> 0
# InputMode.PASSTHROUGH         -> 1
# InputMode.VEL_RAMP            -> 2
# InputMode.POS_FILTER          -> 3
# InputMode.MIX_CHANNELS        -> 4
# InputMode.TRAP_TRAJ           -> 5
# InputMode.TORQUE_RAMP         -> 6
# InputMode.MIRROR              -> 7

CONTROL_MODE = 2
MOTOR_TYPE = 0
INPUT_MODE = 2

# Voltage and current trip levels
odrv.config.dc_bus_overvoltage_trip_level = 38
odrv.config.dc_bus_undervoltage_trip_level = 10.5
odrv.config.dc_max_positive_current = math.inf
odrv.config.dc_max_negative_current = -math.inf
odrv.config.brake_resistor0.enable = False
odrv.axis0.config.motor.motor_type = MOTOR_TYPE
odrv.axis0.config.motor.pole_pairs = 4
odrv.axis0.config.motor.torque_constant = 0.09505747126436781
odrv.axis0.config.motor.current_soft_max = 30
odrv.axis0.config.motor.current_hard_max = 49
odrv.axis0.config.motor.calibration_current = 10
odrv.axis0.config.motor.resistance_calib_max_voltage = 2
odrv.axis0.config.calibration_lockin.current = 10
odrv.axis0.motor.motor_thermistor.config.enabled = True
odrv.axis0.motor.motor_thermistor.config.r_ref = 10000
odrv.axis0.motor.motor_thermistor.config.beta = 3950
odrv.axis0.motor.motor_thermistor.config.temp_limit_lower = 120
odrv.axis0.motor.motor_thermistor.config.temp_limit_upper = 140
odrv.axis0.controller.config.control_mode = CONTROL_MODE
odrv.axis0.controller.config.input_mode = INPUT_MODE
odrv.axis0.controller.config.vel_limit = 10
odrv.axis0.controller.config.vel_limit_tolerance = 1.2
odrv.axis0.config.torque_soft_min = -math.inf
odrv.axis0.config.torque_soft_max = math.inf
odrv.axis0.trap_traj.config.accel_limit = 10
odrv.axis0.controller.config.vel_ramp_rate = 10

# Protocol.SIMPLE     -> 1

# EncoderId.NONE                    -> 0
# EncoderId.INC_ENCODER0            -> 1
# EncoderId.INC_ENCODER1            -> 2
# EncoderId.INC_ENCODER2            -> 3
# EncoderId.SENSORLESS_ESTIMATOR    -> 4
# EncoderId.SPI_ENCODER0            -> 5
# EncoderId.SPI_ENCODER1            -> 6
# EncoderId.SPI_ENCODER2            -> 7
# EncoderId.HALL_ENCODER0           -> 8
# EncoderId.HALL_ENCODER1           -> 9
# EncoderId.RS485_ENCODER0          -> 10
# EncoderId.RS485_ENCODER1          -> 11
# EncoderId.RS485_ENCODER2          -> 12
# EncoderId.ONBOARD_ENCODER0        -> 13
# EncoderId.ONBOARD_ENCODER1        -> 14

# Rs485EncoderMode.DISABLED             -> 0
# Rs485EncoderMode.AMT21_POLLING        -> 1
# Rs485EncoderMode.AMT21_EVENT_DRIVEN   -> 2

CAN_PROTOCOL = 1
ENCODER_ID = 10
RS485_ENCODER_MODE = 2

# Enable and define CAN configuration parameters
odrv.can.config.protocol = CAN_PROTOCOL
odrv.can.config.baud_rate = 250000
odrv.axis0.config.can.node_id = 63
odrv.axis0.config.can.heartbeat_msg_rate_ms = 100
odrv.axis0.config.can.encoder_msg_rate_ms = 0
odrv.axis0.config.can.iq_msg_rate_ms = 0
odrv.axis0.config.can.torques_msg_rate_ms = 0
odrv.axis0.config.can.error_msg_rate_ms = 0
odrv.axis0.config.can.temperature_msg_rate_ms = 0
odrv.axis0.config.can.bus_voltage_msg_rate_ms = 0
odrv.axis0.config.enable_watchdog = False
odrv.axis0.config.load_encoder = ENCODER_ID
odrv.axis0.config.commutation_encoder = ENCODER_ID
odrv.rs485_encoder_group0.config.mode = RS485_ENCODER_MODE

# Don't use uart
odrv.config.enable_uart_a = False

# Check if calibration was successful
if odrv.axis0.current_state != 1:  # Should return to IDLE after calibration
    print(f"{RED}❌ Error: Calibration failed!{RESET}")
    print(f"Current state: {odrv.axis0.current_state}")
    exit()

print(f"{GREEN}✅ Calibration complete.{RESET}")
