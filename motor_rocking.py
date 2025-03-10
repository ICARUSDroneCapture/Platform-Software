import odrive
import time
import math
import csv
import sched

# Motion parameters
amplitude = 10 / 360  # Turns
frequency = .25         # Hz
duration = 12          # Seconds
zero_offset = (7.427) / 360 # IMU reading, will adjust for gearbox

# Setup CSV file
csv_filename = "encoder_data.csv"
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time (s)', 'Setpoint (turns)', 'Encoder Position (turns)'])

print("Finding ODrive...")  
odrv = odrive.find_any()
if not odrv:
    print("Error: ODrive not found!")
    exit()

axis = odrv.axis0

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