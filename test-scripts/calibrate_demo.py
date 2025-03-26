#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import csv
import sched

# ANSI escape codes for colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

# Set your desired offset in degrees
USER_OFFSET_DEGREES = 0  # Change this as needed
USER_OFFSET_TURNS = USER_OFFSET_DEGREES / 360  # Convert to turns

# Motion parameters
amplitude = 10 / 360  # Turns
frequency = .25         # Hz
duration = 12          # Seconds
zero_offset = (7.427) / 360 # IMU reading, will adjust for gearbox

# Connect to ODrive
print("🔌 Connecting to ODrive...")
odrv = odrive.find_any()

print("🛠  Clearing ODrive errors...")
if odrv:
    odrv.clear_errors()
    print(f"{GREEN}✅ ODrive errors cleared.{RESET}")
else:
    print(f"{RED}❌ Error: ODrive not found!{RESET}")
    exit()

axis = odrv.axis0  # Change to `axis1` if needed

# Clear any existing errors
print("🛠  Clearing errors...")
odrv.clear_errors()

# Set motor to IDLE before calibration
print("🛑 Setting motor to IDLE before basic calibration...")
axis.requested_state = 1  # IDLE
time.sleep(1)

# Request calibration
print("🔄 Recalibrating motor...")
axis.requested_state = 3  # Calibration

# Wait for calibration to start
start_time = time.time()
timeout = 10  # Maximum wait time for calibration to start
while axis.current_state not in [3, 7]:  # 3 = Calibration, 7 = Encoder Index Search
    if time.time() - start_time > timeout:
        print(f"{RED}❌ Error: Basic Calibration did not start!{RESET}")
        exit()
    print(f"⏳ Current state: {axis.current_state}")
    time.sleep(1)

print("⚙️ Basic Calibration in progress...")

# Wait for calibration to complete
while axis.current_state in [3, 7]:
    time.sleep(1)

# Check if calibration was successful
if axis.current_state != 1:  # Should return to IDLE after calibration
    print(f"{RED}❌ Error: Basic Calibration failed!{RESET}")
    exit()

print(f"{GREEN}✅ Basic Calibration complete.{RESET}")

# Set control mode to Position Control
print("⚙️ Setting control mode to Position Control...")
axis.controller.config.control_mode = 3  # 3 = Position Control

# Enable Closed-Loop Control
print("🔄 Enabling Closed-Loop Control...")
axis.requested_state = 8  # 8 = Closed-Loop Control
time.sleep(1)

# Verify Closed-Loop Control
if axis.current_state != 8:
    print(f"{RED}❌ Error: Motor did not enter Closed-Loop Control!{RESET}")
    exit()

# Apply the zero position offset
print(f"🎯 Setting motor 0 position to {USER_OFFSET_DEGREES}° offset...")
axis.controller.input_pos = USER_OFFSET_TURNS

print(f"{GREEN}🚀 ODrive setup complete! The motor zero position has been offset by {USER_OFFSET_DEGREES}°.{RESET}")


## ----------------------------------- BASIC CALIBRATION -----------------------------------

# Setup CSV file
csv_filename = "output-csv-data/encoder_data.csv"
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time (s)', 'Setpoint (turns)', 'Encoder Position (turns)'])

# Check if calibration is needed
if axis.current_state == 8:
    print("Motor is already calibrated.")
else:
    print("Starting motor calibration...")
    axis.requested_state = 3                # 3 = Motor Calibration

    timeout = 30
    start_time = time.time()
    while axis.current_state == 3:
        if time.time() - start_time > timeout:
            print("Error: Calibration timed out!")
            exit()
        time.sleep(1)

    if axis.current_state != 1:
        print("Calibration failed or incomplete!")
        exit()

    print("Calibration complete.")

# Enable Closed Loop Control
print("Enabling Closed Loop Control...")
axis.requested_state = 8        # 8 = Closed Loop Control
time.sleep(2)

# Ensure position control is active
axis.controller.config.control_mode = 3     # 3 = Position Control
axis.controller.config.input_mode = 1       # 1 = Pos Filtered

axis.controller.input_pos = zero_offset

# Create scheduler
scheduler = sched.scheduler(time.time, time.sleep)

# Tracking time and scheduling interval
start_time = time.time()
interval = 0.014  # 14ms interval
end_time = start_time + duration

def update_position():
    current_time = time.time() - start_time
    if current_time >= duration:
        print("Motion complete.")
        return
    
    # Generate sinusoidal setpoint
    setpoint = zero_offset + amplitude * math.sin(2 * math.pi * frequency * current_time)
    axis.controller.input_pos = setpoint * 50

    # Read encoder position
    encoder_position = axis.pos_estimate * 50

    # Write to CSV
    with open(csv_filename, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([current_time, setpoint*50, encoder_position/50])

    # Schedule next update
    scheduler.enter(interval, 1, update_position)

# Start sinusoidal motion
print("Starting sinusoidal motion...")
scheduler.enter(0, 1, update_position)
scheduler.run()

print('------ Basic Calibration and Rocking Test Complete, Now Custom Calibration ------')

## ----------------------------------- CUSTOM CALIBRATION -----------------------------------

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

CONTROL_MODE = ControlMode.VELOCITY_CONTROL
MOTOR_TYPE = MotorType.HIGH_CURRENT
INPUT_MODE = InputMode.VEL_RAMP

# # CONTROL_MODE = 2
# # MOTOR_TYPE = 0
# # INPUT_MODE = 2

# # Voltage and current trip levels
# odrv.config.dc_bus_overvoltage_trip_level = 38
# odrv.config.dc_bus_undervoltage_trip_level = 10.5
# odrv.config.dc_max_positive_current = math.inf
# odrv.config.dc_max_negative_current = -math.inf
# odrv.config.brake_resistor0.enable = False
axis.config.motor.motor_type = MOTOR_TYPE
# axis.config.motor.pole_pairs = 4
# axis.config.motor.torque_constant = 0.09505747126436781
# axis.config.motor.current_soft_max = 30
# axis.config.motor.current_hard_max = 49
# axis.config.motor.calibration_current = 10
# axis.config.motor.resistance_calib_max_voltage = 2
# axis.config.calibration_lockin.current = 10
# axis.motor.motor_thermistor.config.enabled = True
# axis.motor.motor_thermistor.config.r_ref = 10000
# axis.motor.motor_thermistor.config.beta = 3950
# axis.motor.motor_thermistor.config.temp_limit_lower = 120
# axis.motor.motor_thermistor.config.temp_limit_upper = 140
# axis.controller.config.control_mode = CONTROL_MODE
axis.controller.config.input_mode = INPUT_MODE
# axis.controller.config.vel_limit = 10
# axis.controller.config.vel_limit_tolerance = 1.2
# axis.config.torque_soft_min = -math.inf
# axis.config.torque_soft_max = math.inf
# axis.trap_traj.config.accel_limit = 10
# axis.controller.config.vel_ramp_rate = 10

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


CAN_PROTOCOL = Protocol.SIMPLE
ENCODER_ID = EncoderId.RS485_ENCODER0
RS485_ENCODER_MODE = Rs485EncoderMode.AMT21_EVENT_DRIVEN

# CAN_PROTOCOL = 1
# ENCODER_ID = 10
# RS485_ENCODER_MODE = 2

# Enable and define CAN configuration parameters
# odrv.can.config.protocol = CAN_PROTOCOL
odrv.can.config.protocol = CAN_PROTOCOL
odrv.can.config.baud_rate = 250000
axis.config.can.node_id = 63
axis.config.can.heartbeat_msg_rate_ms = 100
axis.config.can.encoder_msg_rate_ms = 0
axis.config.can.iq_msg_rate_ms = 0
axis.config.can.torques_msg_rate_ms = 0
axis.config.can.error_msg_rate_ms = 0
axis.config.can.temperature_msg_rate_ms = 0
axis.config.can.bus_voltage_msg_rate_ms = 0
axis.config.enable_watchdog = False
axis.config.load_encoder = ENCODER_ID
axis.config.commutation_encoder = ENCODER_ID
odrv.rs485_encoder_group0.config.mode = RS485_ENCODER_MODE

# Don't use uart
odrv.config.enable_uart_a = False

# --------------------------------------------------------------------------------
# Axis States
# --------------------------------------------------------------------------------

# Again, not a type, number 5 is skipped

# AxisState.UNDEFINED                           -> 0
# AxisState.IDLE                                -> 1
# AxisState.STARTUP_SEQUENCE                    -> 2
# AxisState.FULL_CALIBRATION_SEQUENCE           -> 3
# AxisState.MOTOR_CALIBRATION                   -> 4
# AxisState.ENCODER_INDEX_SEARCH                -> 6
# AxisState.ENCODER_OFFSET_CALIBRATION          -> 7
# AxisState.CLOSED_LOOP_CONTROL                 -> 8
# AxisState.LOCKIN_SPIN                         -> 9
# AxisState.ENCODER_DIR_FIND                    -> 10
# AxisState.HOMING                              -> 11
# AxisState.ENCODER_HALL_POLARITY_CALIBRATION   -> 12
# AxisState.ENCODER_HALL_PHASE_CALIBRATION      -> 13
# AxisState.ANTICOGGING_CALIBRATION             -> 14

# Check if calibration was successful
if axis.current_state != 8:  # Should return to IDLE after calibration
    print(f"{RED}❌ Error: CUSTOM Calibration failed!{RESET}")
    print(f"Current state: {axis.current_state}")
    exit()

print(f"{GREEN}✅ CUSTOM Calibration complete. Using Closed Loop Control{RESET}")

## --------------------------------- BASIC ROCKING TEST DONE, NOW CONTINUOUS ---------------------------------

# Calibrate motor and wait for it to finish
print("starting calibration...")
axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
while axis.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# To read a value, simply read the property
print("Bus voltage is " + str(odrv.vbus_voltage) + "V")

# Or to change a value, just assign to the property
axis.controller.input_pos = 3.14
print("Position setpoint is " + str(axis.controller.pos_setpoint))

# And this is how function calls are done:
for i in [1,2,3,4]:
    print('voltage on GPIO{} is {} Volt'.format(i, odrv.get_adc_voltage(i)))

# A sine wave to test
t0 = time.monotonic()
while True:
    setpoint = 4.0 * math.sin((time.monotonic() - t0)*2)
    print("goto " + str(int(setpoint)))
    axis.controller.input_pos = setpoint
    time.sleep(0.01)

# Some more things you can try:

# Write to a read-only property:
odrv.vbus_voltage = 11.0  # fails with `AttributeError: can't set attribute`

# Assign an incompatible value:
odrv.motor0.pos_setpoint = "I like trains"  # fails with `ValueError: could not convert string to float`
