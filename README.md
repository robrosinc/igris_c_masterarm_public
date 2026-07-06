# IGRIS_C_MASTERARM

ROS2 package for MasterArm teleoperation of IGRIS-C. It provides:
- a headless relay node for direct MasterArm-to-robot publishing
- an SDK-style GUI example client, adapted for MasterArm workflows

## Overview

This package reads joint positions from MasterArm devices using the MasterArm SDK and converts them into control commands for both robot arms and hands. The GUI tool also subscribes to `rt/lowstate` and shows the current robot arm positions against the MasterArm-driven command values.

## Requirements

Hardware
- MasterArm device (Dynamixel-based)
- USB-2-Serial adapter (/dev/ttyUSB0)

## Installation
0. submodules
``` bash
git submodule update --init --recursive
```

1. Initialize Submodules

```bash
cd ~/ros2_ws/src/igris_c_masterarm
git submodule update --init --recursive
```

2. Install DynamixelSDK

```bash
sudo apt-get install ros-${ROS_DISTRO}-dynamixel-sdk
# Or build from source:
cd ~/ros2_ws/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd ~/ros2_ws
colcon build --packages-select dynamixel_sdk
```

3. Install OpenSSL
``` bash
sudo apt install libssl-dev
```

4. Stage third-party artefacts

Before building either the ROS node or the GUI example, stage the dependency
artefacts that this repository expects:

```bash
cd ~/ros2_ws/src/igris_c_masterarm
bash scripts/build_deps.sh
```

If you use `task`, the equivalent command is:

```bash
cd ~/ros2_ws/src/igris_c_masterarm
task deps
```

This creates:
- `third_party/MasterArmSDK/`
- `third_party/igris_c_sdk_public/`

Without this step, CMake will fail because paths such as
`third_party/MasterArmSDK/include` do not exist yet.

### 5. Build ROS node

```bash
cd ~/ros2_ws
colcon build --packages-select igris_c_masterarm
source install/setup.bash
```

Use this when you want to run `igris_c_masterarm_node` or the ROS launch file.

### 6. Build GUI example

```bash
cd ~/ros2_ws/src/igris_c_masterarm/examples
./build.sh
```

The GUI example reads the staged SDK directories from:
- `third_party/igris_c_sdk_public/`
- `third_party/MasterArmSDK/`

It does not require `igris_c_masterarm` itself to be built first.

## Usage
Headless relay node
```bash
ros2 run igris_c_masterarm igris_c_masterarm_node
```

GUI example build
```bash
cd ~/ros2_ws/src/igris_c_masterarm/examples
./build.sh
```

The GUI example is pinned to:
`igris_c_masterarm/third_party/igris_c_sdk_public`
It does not fall back to `COLCON_PREFIX_PATH` or any SDK installed from the robot-side workspace.

GUI example run
```bash
cd ~/ros2_ws/src/igris_c_masterarm/examples/build
./masterarm_gui_client /dev/ttyUSB0 1000000 0
./masterarm_gui_client /dev/ttyUSB0 1000000 0 robot1
```

Launch file for the headless relay node
```bash
ros2 launch igris_c_masterarm igris_c_masterarm.launch.py port:="/dev/ttyUSB0" baud:=1000000
ros2 launch igris_c_masterarm igris_c_masterarm.launch.py port:="/dev/ttyUSB0" baud:=1000000 namespace:=robot1
```

Pre-execution Checklist

1. Verify MasterArm device is connected to `/dev/ttyUSB0`
2. Verify IGRIS-C robot is running
3. For the GUI example, use the Start/Stop button to enable or disable command publishing

## DDS Topics

> This package does not publish finger commands on standard ROS2 topics. It uses CycloneDDS-style custom DDS topics for IGRIS communication.

Published Topics

- `rt/lowcmd`: Robot motor control commands (`LowCmd`)
- `rt/handcmd`: Hand motor control commands (`HandCmd`)

Subscribed Topics

- `rt/lowstate`: Robot current state (`LowState`)
- `rt/robotstate`: Robot control state (`RobotState`)

## ROS2 Launch Parameters
| **parameter name** | **default**  | **description**                                        |
|--------------------|--------------|--------------------------------------------------------|
| **port**           | /dev/ttyUSB0 | USB-serial port to communicate with the MasterArm      |
| **baud**           | 1000000      | USB-serial port baudrate                               |
| **domain_id**      | 0            | CycloneDDS domain id                                   |
| **namespace**      | empty        | Optional DDS namespace prefix                          |

## GUI Behavior

The GUI example:
- removes the slider-based command editor from `sdk_gui_client`
- shows MasterArm command values and the current `rt/lowstate` arm motor values side by side
- does not gate publishing on `RobotState`
- starts and stops low-level command publishing only with the GUI button
- keeps service buttons for torque, BMS, hand init, and control-mode switching

## Trouble Shooting

Serial Port Permissions

```bash
sudo chmod 666 /dev/ttyUSB0
# or add user to dialout group
sudo usermod -aG dialout $USER
```
