# ICARUS Arm Control Law - ROS2 Implementation

A ROS 2 wrapper node implementation of the 3DOF control law for the ICARUS Drone Capture robotic arm.

## Setup

### Important

### Important

**Firmware Version** - The IMX/uINS should be updated with the latest firmware found on the Inertial Sense [release page](https://github.com/inertialsense/inertial-sense-sdk/releases).  Download the appropriate `.hex` file and use the Inertial Sense EvalTool, CLTool, or SDK to upload the firmware.

**Dialout Group** - The user must be a member of the `dialout` group, or the user won't have access to the serial port.

## Execution

```bash
ros2 run arm_control imu_talker
ros2 run arm_control imu_listener
```

fill in...