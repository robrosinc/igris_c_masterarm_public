# IGRIS-C MasterArm

Teleoperation package for the IGRIS-C robot. A MasterArm device on a USB-serial
port is read at a fixed rate and its joint targets are published to the robot
over Cyclone DDS.

This tree is a ready-to-build ROS 2 package. Its third-party dependencies
(`third_party/igris_c_sdk_public/`, `third_party/MasterArmSDK/`) are already
staged inside it — there is nothing to fetch or assemble first.

## Requirements

- ROS 2 (Humble or Jazzy) with `colcon`
- `ros-${ROS_DISTRO}-dynamixel-sdk`
- `libssl-dev`

```bash
sudo apt install ros-${ROS_DISTRO}-dynamixel-sdk libssl-dev
```

## Build

Copy this package into your workspace `src/` and build it:

```bash
cp -r igris_c_masterarm ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select igris_c_masterarm
source install/setup.bash
```

The GUI example builds on its own and does not need the package built first:

```bash
cd ~/ros2_ws/src/igris_c_masterarm/examples
./build.sh
```

## Run

Headless relay node:

```bash
ros2 run igris_c_masterarm igris_c_masterarm_node
```

Launch file, with the parameters that pair it with a robot:

```bash
ros2 launch igris_c_masterarm igris_c_masterarm.launch.py \
    port:=/dev/ttyUSB0 baud:=1000000 domain_id:=27 namespace:=igris_c_IG27
```

GUI example:

```bash
cd ~/ros2_ws/src/igris_c_masterarm/examples/build
./masterarm_gui_client /dev/ttyUSB0 1000000 27 igris_c_IG27
```

Before running: the MasterArm must be on the serial port given by `port`, and
the robot must be running. In the GUI, command publishing starts and stops with
the Start/Stop button.

## Pairing with the robot

Three settings must match the robot or the two never discover each other. A
mismatch is silent — no error, just nothing received:

| Setting | Must equal the robot's |
| --- | --- |
| `domain_id` | `igris_c.network.cyclonedds.domain_id` |
| `namespace` | the robot's resolved DDS namespace (e.g. `igris_c_IG27`) |
| `dds_topic_naming` | layout half of `igris_c.network.transport` |

`dds_topic_naming` selects the wire layout: `native` (default) publishes
`<namespace>/<topic>`, `ros_compatible` publishes `rt/<namespace>/<topic>`,
which is what a ROS 2 node sees. Both sides must use the same one.

The startup banner prints all three, so a run that receives nothing can be
checked against the robot at a glance:

```
=== IGRIS-C Masterarm Node (Cyclone DDS) ===
domain id      : 27
namespace      : igris_c_IG27
topic naming   : native
port           : /dev/ttyUSB0
baud           : 1000000
```

## Network interface (CYCLONEDDS_URI)

DDS discovery needs a Cyclone DDS XML naming the interface that reaches the
robot. A sample `cyclonedds.xml` ships at the root of this package; set its
`<NetworkInterface name="...">` to the interface on this machine (`ip link`),
then point Cyclone DDS at it before launching:

```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```

That file configures the network interface only. The domain id and namespace
are separate, and are passed as launch arguments as shown above.

## DDS topics

This package does not use standard ROS 2 topics for the robot data contract; it
publishes and subscribes Cyclone DDS topics directly.

Published:

- `lowcmd` — arm motor commands (`LowCmd_`)
- `handcmd` — hand motor commands (`HandCmd_`)

Subscribed:

- `lowstate` — robot state (`LowState_`)
- `robotstate` — robot control state (`RobotState_`)
- `handstate` — hand state (`HandState_`, GUI example only)

## Launch parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | USB-serial port of the MasterArm |
| `baud` | `1000000` | Serial baud rate |
| `domain_id` | `0` | Cyclone DDS domain id |
| `namespace` | empty | DDS namespace prefix |
| `dds_topic_naming` | `native` | Wire layout: `native` or `ros_compatible` |

The GUI example takes the same settings as positional arguments:
`masterarm_gui_client <port> <baud> [domain_id] [namespace] [dds_topic_naming]`.

## GUI behaviour

- Shows the MasterArm command values and the robot's current `lowstate` arm
  motor values side by side
- Publishing of low-level commands starts and stops only with the GUI button
- Keeps the service buttons for torque, PDU, hand init and control-mode switching
- Does not gate publishing on `RobotState_`

## Troubleshooting

**Nothing is received from the robot.** Check the three pairing settings above
against the robot's configuration; the startup banner prints what this side is
using. Then confirm `CYCLONEDDS_URI` names an interface that reaches the robot.

**Serial port permission denied.**

```bash
sudo chmod 666 /dev/ttyUSB0
# or, once, add yourself to the dialout group and log back in
sudo usermod -aG dialout $USER
```
