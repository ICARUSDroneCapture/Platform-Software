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
odrv = odrv0

print("Clearing ODrive errors...")
if odrv:
    odrv.clear_errors()
    print(f"{GREEN}✅ ODrive errors cleared.{RESET}")
else:
    print(f"{RED}❌ Error: ODrive not found!{RESET}")
    exit()

# Voltage and current trip levels
odrv.config.dc_bus_overvoltage_trip_level = 38
odrv.config.dc_bus_undervoltage_trip_level = 10.5
odrv.config.dc_max_positive_current = math.inf
odrv.config.dc_max_negative_current = -math.inf
odrv.config.brake_resistor0.enable = False
odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
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
odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrv.axis0.controller.config.vel_limit = 10
odrv.axis0.controller.config.vel_limit_tolerance = 1.2
odrv.axis0.config.torque_soft_min = -math.inf
odrv.axis0.config.torque_soft_max = math.inf
odrv.axis0.trap_traj.config.accel_limit = 10
odrv.axis0.controller.config.vel_ramp_rate = 10

# Enable and define CAN configuration parameters
odrv.can.config.protocol = Protocol.SIMPLE
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
odrv.axis0.config.load_encoder = EncoderId.RS485_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.RS485_ENCODER0
odrv.rs485_encoder_group0.config.mode = Rs485EncoderMode.AMT21_EVENT_DRIVEN

# Don't use uart
odrv.config.enable_uart_a = False

# Check if calibration was successful
if odrv.axis0.current_state != 1:  # Should return to IDLE after calibration
    print(f"{RED}❌ Error: Calibration failed!{RESET}")
    print(f"Current state: {odrv.axis0.current_state}")
    exit()

print(f"{GREEN}✅ Calibration complete.{RESET}")