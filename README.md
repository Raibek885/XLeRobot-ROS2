# XLeRobot ROS 2

ROS 2 Jazzy teleoperation stack for the XLeRobot dual-arm mobile manipulator. The
repository contains only the ROS packages, the required XLeRobot hardware adapters,
configuration, tests, and documentation. Clone it directly into a workspace `src/`
directory.

## Packages

- `xlerobot_driver`: owns the two serial buses, publishes `sensor_msgs/JointState`
  and base velocity, and accepts `JointState`, `JointTrajectory`, and `Twist`
  commands. It supports the three-wheel, differential two-wheel, and mecanum
  XLeRobot variants plus a no-hardware mock mode. The required LeRobot-compatible
  XLeRobot adapters are bundled in this package.
- `xlerobot_teleop`: deadman-protected Xbox-style gamepad control and a generic,
  name-based leader/follower relay with scaling, offsets, filtering, a step bound,
  and a stale-input timeout.
- `xlerobot_bringup`: launch files and conservative default parameter files.

The design is inspired by
[`legalaspro/so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai),
adapted for XLeRobot's dual arms, head, and mobile base.

## Prerequisites

- Ubuntu 24.04 and ROS 2 Jazzy.
- LeRobot installed with Feetech support in the same Python environment used to
  build this workspace. Verify it with `python3 -c "import lerobot"`.
- A calibrated XLeRobot. Complete the standard LeRobot motor setup and calibration
  before starting the ROS driver.
- Never run calibration while the robot is free to collide with people or itself.
  `calibrate_on_connect` defaults to `false`.

## Build

Install LeRobot with Feetech support first. Use the same Python 3.12 environment
for LeRobot and ROS 2 Jazzy:

```bash
git clone https://github.com/huggingface/lerobot.git ~/lerobot
cd ~/lerobot
pip install -e ".[feetech]"
python3 -c "import lerobot"
```

Then create the ROS workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Raibek885/XLeRobot-ROS2.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to xlerobot_bringup
source install/setup.bash
```

Run a hardware-free smoke test first:

```bash
ros2 launch xlerobot_bringup joy_teleop.launch.py mock_hardware:=true
```

Then launch the real robot after checking port names and lifting the drive wheels:

```bash
ros2 launch xlerobot_bringup joy_teleop.launch.py \
  port1:=/dev/ttyACM0 port2:=/dev/ttyACM1 \
  robot_variant:=xlerobot robot_id:=my_xlerobot
```

Other variants are `xlerobot_2wheels` and `xlerobot_mecanum`. Stable udev aliases
are strongly recommended instead of relying on `ttyACM` enumeration order.
Use the same `robot_id` that owns the existing LeRobot calibration file. Only set
`calibrate_on_connect:=false` in normal launches. If no calibration exists, run the
interactive CLI from a terminal with the wheels lifted and free space around both
arms:

```bash
ros2 run xlerobot_driver xlerobot_calibrate \
  --variant xlerobot \
  --robot-id my_xlerobot \
  --port1 /dev/ttyACM0 \
  --port2 /dev/ttyACM1
```

## Xbox-style controls

- Hold **A**: drive with the left stick; right-stick horizontal turns the base.
- Hold **A + B**: turbo drive.
- Hold **LB**: left arm. Hold **RB**: right arm.
- Hold **Start/Menu**: head pan/tilt.
- In arm mode, axes `0/1/4/3/6/7` control shoulder pan, shoulder lift, elbow,
  wrist flex, wrist roll, and gripper. Axis numbering varies between controllers;
  inspect `ros2 topic echo /joy` and edit
  `xlerobot_bringup/config/joy_teleop.yaml` if needed.

Releasing the drive deadman publishes zero velocity. Missing joystick messages and
stale `cmd_vel` input independently stop the base after 250 ms.

## Competition trajectory macro

`joy_teleop.launch.py` also starts a guarded recorder/player for a repeatable
bimanual task. It records the ten arm joints but deliberately excludes both
grippers, so returning to the recorded start pose cannot release held objects.

1. Grasp both objects at repeatable marked depths and manually move the arms to a
   safe pre-insertion pose (`task_ready`).
2. Hold **B** and tap **Y** to start recording. The first captured frame becomes
   `task_ready`.
3. Perform the task with normal **LB**/**RB** teleoperation.
4. Hold **B** and tap **Y** again to save the macro.
5. On later attempts, grasp the objects, move the arms roughly near `task_ready`,
   and tap **Y**. The robot takes four seconds to reach the exact recorded start,
   waits briefly, and replays the task.

The macro is stored at `~/.ros/xlerobot_macros/connect_sticks.json`. Replay is
refused if any controlled joint is more than 35 degrees from `task_ready`.
Pressing **B**, **A**, **LB**, **RB**, or **Start/Menu**, or losing the gamepad
connection, cancels replay immediately. Start with the wheels lifted and keep a
hand near power while validating a new macro. Set `enable_macro:=false` on the
launch command to disable this feature.

### Arm holding stiffness

The driver defaults to the compliant `arm_p_coefficient:=16`. After confirming
stable serial communication under load, test 24 first and only then the STS3215
factory stiffness of 32:

```bash
ros2 launch xlerobot_bringup joy_teleop.launch.py arm_p_coefficient:=24 ...
ros2 launch xlerobot_bringup joy_teleop.launch.py arm_p_coefficient:=32 ...
```

This changes position-loop stiffness, not the configured torque/current safety
limits. Stop immediately if an arm chatters, oscillates, overheats, or produces
communication errors under load.

The default hardware state and command rates are intentionally conservative at
15 Hz and 30 Hz. They can be reduced further for noisy or long Feetech buses:

```bash
ros2 launch xlerobot_bringup joy_teleop.launch.py \
  state_publish_rate:=10.0 command_rate:=20.0 ...
```

## Leader/follower

```bash
ros2 launch xlerobot_bringup leader_follower.launch.py mock_hardware:=true
```

The default leader topic is `/leader/joint_states`. Edit
`xlerobot_bringup/config/leader_follower.yaml` when leader joint names, directions,
or zero offsets differ. All positions are ROS-standard radians; grippers use
normalized `[0, 1]` opening. The hardware bridge converts revolute joints to
degrees and grippers to the existing driver's percentage convention.

## Interfaces

| Direction | Topic/service | Type |
| --- | --- | --- |
| Output | `joint_states` | `sensor_msgs/msg/JointState` |
| Output | `base_velocity` | `geometry_msgs/msg/TwistStamped` |
| Input | `joint_commands` | `sensor_msgs/msg/JointState` (partial commands allowed) |
| Input | `joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` (last point used) |
| Input | `cmd_vel` | `geometry_msgs/msg/Twist` |
| Service | `enable` | `std_srvs/srv/SetBool` |

Calling `enable` with `false` suppresses commands and immediately stops the base.
This is a software stop, not a substitute for a physical emergency stop.

## Acknowledgements

The hardware adapters are derived from the Apache-2.0 licensed
[`Vector-Wangel/XLeRobot`](https://github.com/Vector-Wangel/XLeRobot) and LeRobot
implementations. The ROS package organization was informed by
[`legalaspro/so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai).
