import odrive
import time

# ANSI escape codes for colors
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

# Set your desired offset in degrees
USER_OFFSET_DEGREES = -3.337+1.226  # Change this as needed
USER_OFFSET_TURNS = USER_OFFSET_DEGREES / 360  # Convert to turns

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
print("🛑 Setting motor to IDLE before calibration...")
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
        print(f"{RED}❌ Error: Calibration did not start!{RESET}")
        exit()
    print(f"⏳ Current state: {axis.current_state}")
    time.sleep(1)

print("⚙️ Calibration in progress...")

# Wait for calibration to complete
while axis.current_state in [3, 7]:
    time.sleep(1)

# Check if calibration was successful
if axis.current_state != 1:  # Should return to IDLE after calibration
    print(f"{RED}❌ Error: Calibration failed!{RESET}")
    exit()

print(f"{GREEN}✅ Calibration complete.{RESET}")

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