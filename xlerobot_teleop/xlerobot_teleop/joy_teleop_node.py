"""Xbox-style gamepad teleoperation for the XLeRobot base, arms and head."""

from math import pi

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy

from .mapping import axis_value, button_pressed, clamp


LEFT_JOINTS = [
    "left_arm_shoulder_pan",
    "left_arm_shoulder_lift",
    "left_arm_elbow_flex",
    "left_arm_wrist_flex",
    "left_arm_wrist_roll",
    "left_arm_gripper",
]
RIGHT_JOINTS = [name.replace("left_", "right_") for name in LEFT_JOINTS]
HEAD_JOINTS = ["head_motor_1", "head_motor_2"]


class JoyTeleop(Node):
    """Translate Joy into deadman-protected base and incremental joint commands."""

    def __init__(self) -> None:
        super().__init__("xlerobot_joy_teleop")
        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("joint_command_topic", "joint_commands")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("joy_timeout", 0.25)
        self.declare_parameter("deadzone", 0.12)
        self.declare_parameter("base_deadman_button", 0)
        self.declare_parameter("turbo_button", 1)
        self.declare_parameter("left_arm_mode_button", 4)
        self.declare_parameter("right_arm_mode_button", 5)
        self.declare_parameter("head_mode_button", 7)
        self.declare_parameter("linear_x_axis", 1)
        self.declare_parameter("linear_y_axis", 0)
        self.declare_parameter("angular_z_axis", 3)
        self.declare_parameter("max_linear_speed", 0.20)
        self.declare_parameter("max_angular_speed", 1.05)
        self.declare_parameter("turbo_multiplier", 1.5)
        self.declare_parameter("joint_rate", 0.70)
        self.declare_parameter("gripper_rate", 0.65)
        self.declare_parameter("head_rate", 0.70)
        self.declare_parameter("arm_axes", [0, 1, 3, 2, 6, 7])
        self.declare_parameter("arm_axis_directions", [1, 1, 1, 1, 1, 1])
        self.declare_parameter("head_axes", [0, 1])

        rate = float(self.get_parameter("publish_rate").value)
        deadzone = float(self.get_parameter("deadzone").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        if not 0.0 <= deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")
        self._arm_axes = [int(value) for value in self.get_parameter("arm_axes").value]
        self._arm_axis_directions = [
            int(value) for value in self.get_parameter("arm_axis_directions").value
        ]
        self._head_axes = [int(value) for value in self.get_parameter("head_axes").value]
        if (
            len(self._arm_axes) != 6
            or len(self._arm_axis_directions) != 6
            or len(self._head_axes) != 2
        ):
            raise ValueError(
                "arm_axes and arm_axis_directions must contain 6 entries; "
                "head_axes must contain 2"
            )
        if any(direction not in (-1, 1) for direction in self._arm_axis_directions):
            raise ValueError("arm_axis_directions entries must be -1 or 1")

        self._cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self._joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_command_topic").value), 10
        )
        self.create_subscription(
            Joy,
            str(self.get_parameter("joy_topic").value),
            self._on_joy,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / rate, self._tick)

        self._joy: Joy | None = None
        self._state: dict[str, float] = {}
        self._targets: dict[str, float] = {}
        self._active_group: str | None = None
        now = self.get_clock().now()
        self._last_joy_time = now
        self._last_tick_time = now
        self._stale_stop_sent = True
        self._warned_no_state = False

        limit = pi
        self._limits = {
            **{name: (-limit, limit) for name in LEFT_JOINTS + RIGHT_JOINTS},
            "left_arm_gripper": (0.0, 1.0),
            "right_arm_gripper": (0.0, 1.0),
            "head_motor_1": (-pi / 2.0, pi / 2.0),
            "head_motor_2": (-pi / 2.0, pi / 2.0),
        }
        self.get_logger().info(
            "Joy teleop ready: hold A for base, LB/RB for arms, Start for head"
        )

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg
        self._last_joy_time = self.get_clock().now()
        self._stale_stop_sent = False

    def _on_joint_state(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            self.get_logger().warning("Ignoring malformed JointState")
            return
        self._state.update(zip(msg.name, msg.position))

    def _button(self, parameter: str) -> bool:
        assert self._joy is not None
        index = int(self.get_parameter(parameter).value)
        return button_pressed(self._joy.buttons, index)

    def _axis(self, index_parameter: str) -> float:
        assert self._joy is not None
        index = int(self.get_parameter(index_parameter).value)
        deadzone = float(self.get_parameter("deadzone").value)
        return axis_value(self._joy.axes, index, deadzone)

    def _selected_group(self) -> str | None:
        if self._button("head_mode_button"):
            return "head"
        if self._button("left_arm_mode_button"):
            return "left"
        if self._button("right_arm_mode_button"):
            return "right"
        return None

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = min(0.1, max(0.0, (now - self._last_tick_time).nanoseconds / 1e9))
        self._last_tick_time = now
        age = (now - self._last_joy_time).nanoseconds / 1e9
        if self._joy is None or age > float(self.get_parameter("joy_timeout").value):
            if not self._stale_stop_sent:
                self._cmd_vel_pub.publish(Twist())
                self._stale_stop_sent = True
            self._active_group = None
            return

        group = self._selected_group()
        if group is not None:
            self._cmd_vel_pub.publish(Twist())
            self._update_joints(group, dt)
        else:
            self._publish_base()
        self._active_group = group

    def _publish_base(self) -> None:
        msg = Twist()
        if self._button("base_deadman_button"):
            multiplier = (
                float(self.get_parameter("turbo_multiplier").value)
                if self._button("turbo_button")
                else 1.0
            )
            msg.linear.x = (
                self._axis("linear_x_axis")
                * float(self.get_parameter("max_linear_speed").value)
                * multiplier
            )
            msg.linear.y = (
                self._axis("linear_y_axis")
                * float(self.get_parameter("max_linear_speed").value)
                * multiplier
            )
            msg.angular.z = (
                self._axis("angular_z_axis")
                * float(self.get_parameter("max_angular_speed").value)
                * multiplier
            )
        self._cmd_vel_pub.publish(msg)

    def _update_joints(self, group: str, dt: float) -> None:
        if group == "head":
            joints = HEAD_JOINTS
        elif group == "left":
            joints = LEFT_JOINTS
        else:
            joints = RIGHT_JOINTS
        if group != self._active_group or any(name not in self._targets for name in joints):
            missing = [name for name in joints if name not in self._state]
            if missing:
                if not self._warned_no_state:
                    self.get_logger().warning(
                        "Arm/head teleop waits for joint_states; missing: "
                        + ", ".join(missing)
                    )
                    self._warned_no_state = True
                return
            self._targets.update({name: self._state[name] for name in joints})
            self._warned_no_state = False

        if any(name not in self._targets for name in joints):
            return

        deadzone = float(self.get_parameter("deadzone").value)
        if group == "head":
            axes = self._head_axes
            directions = [1, 1]
            rate = float(self.get_parameter("head_rate").value)
            rates = [rate, rate]
        else:
            axes = self._arm_axes
            directions = self._arm_axis_directions
            joint_rate = float(self.get_parameter("joint_rate").value)
            rates = [joint_rate] * 5 + [float(self.get_parameter("gripper_rate").value)]

        for name, axis, direction, rate in zip(joints, axes, directions, rates):
            delta = axis_value(self._joy.axes, axis, deadzone) * direction * rate * dt
            lower, upper = self._limits[name]
            self._targets[name] = clamp(self._targets[name] + delta, lower, upper)

        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = joints
        command.position = [self._targets[name] for name in joints]
        self._joint_pub.publish(command)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
