# Platform-Software

Onboard platform stabilization software.

## Usage

#### Background

This is all tested on Noble Ubuntu (Ubuntu 24.04.2). Likely should work for other versions of Ubuntu, but untested.
If not using Ubuntu, it is recommended to use VirtualBox to run a Noble Ubuntu VM to be able to build code if there is no access to hardware.
> [!IMPORTANT]
> When creating a VirtualBox VM, be sure to use at least 4Gs of RAM, recommended 8Gs. Trying to build ROS with any less RAM will probably cause a crash. Same goes for the hardware a user chooses to build on.
> 
> Disk space is up to preference, I allocated 50Gs to not worry. Processor count up to preference, especially dependening on how many packages you may want to build at once when running colcon. I assigned 4 processors.
> 
> GuestAdditions in recommended to improve QoL when using the VM.

Code is built and run on a Raspberry Pi 5 with 16G of RAM, connected to an Inertial Sense IMU (IMX-5) and ODrive Motors (ODRIVE S1).

#### Setup

Follow ROS2 installation instructions:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Make ROS workspace in home directory (or wherever you choose), and make `src` directory:
```
mkdir ~/ros2_ws
cd ~/ros2_ws
mkdir src
```
Clone the Platform Software repo into the `src` direction, and checkout the `arm-control-implementation` branch:
```
cd src
git clone git@github.com:ICARUSDroneCapture/Platform-Software.git
cd Platform-Software
git checkout arm-control-implementation
```

Add symlink in `src` for the icarus-arm-control package:
```
cd ~/ros2_ws/src
sudo ln -s Platform-Software/icarus-arm-control/ROS/ros2 icarus_arm_control
```
## Building and Running

For ease of use, add the following code to the end of your `~/.bashrc` file using your editor of choice, such as `vim`:
```
# ROS Tools
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

alias sourceICARUS="cd ~/ros2_ws; source install/setup.bash"
alias motorEnv="source ~/motor_venv/bin/activate"
alias colconClean="sudo rm -rf ~/ros2_ws/log; rm -rf ~/ros2_ws/build; rm -rf ~/ros2_ws/install"

```
You may have already put the first two lines (the `source /opt/ros/jazzy/setup.bash` and `export ROS_DOMAIN_ID=0`) into the `.bashrc` file during the ROS2 installation process, in which case they can be ignored.

#### Building

From the `~/ros2_ws` directory, run `colcon build`. It will throw many warnings, but the package should finish building:
```
cd ~/ros2_ws
colcon build
```
If you want to do a fresh build, run `colconClean` from the `ros2_ws` directory and build again:
```
cd ~/ros2_ws
colconClean
colcon build
```
Be careful that you're in the correct directory when you run `colconClean`. It runs a forced recursive directory remove command, which could delete something unwanted if in the wrong place. For that reason, it required sudo privileges.

#### Running

After building successfully, you can run the icarus package controller node. To do so, run the following commands:
```
sourceICARUS
ros2 run icarus_arm_control controller_listener_node
```

## Installation Notes

##### MatPlotPlusPlus

User must install MatPlotPlusPlus: (live plotting cpp package)
- https://github.com/alandefreitas/matplotplusplus
- Recommended to use CMake to install system wide (or local wide), (section "Install as a Package via CMake" in the README)

##### libusb

User must install libusb:
` sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev `

If `colcon build` still throws the "install error "config.h: No such file" error, nagivate to `Platform-Software/icarus-arm-control/src`. Remove the `libusb` folder, and clone the following repo directionly into `src` and checkout the following branch: https://github.com/inertialsense/libusb/tree/1c2ddf4f19caa222161162fb5aa28f54a3980f94

```
cd ~/ros2_ws/src/Platform-Software/icarus-arm-control/src
rm -rf libusb
git clone git@github.com:inertialsense/libusb.git
cd libusb
git checkout 1c2ddf4f19caa222161162fb5aa28f54a3980f94
```

If you still get the "install error "config.h: No such file" error when building, navigate to `/usr/include` and copy the `libusb