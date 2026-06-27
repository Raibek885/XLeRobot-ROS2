"""Safe name-based leader-to-XLeRobot joint mirroring."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .mapping import extract_positions, filtered_step, mapped_positions


DEFAULT_JOINTS = [
    "left_arm_shoulder_pan",
    "left_arm_shoulder_lift",
    "left_arm_elbow_flex",
    "left_arm_wrist_flex",
    "left_arm_wrist_roll",
    "left_arm_gripper",
    "right_arm_shoulder_pan",
    "right_arm_shoulder_lift",
    "right_arm_elbow_flex",
    "right_arm_wrist_flex",
    "right_arm_wrist_roll",
    "right_arm_gripper",
]


class LeaderFollowerTeleop(Node):
    """Relay selected leader joints to the driver's partial JointState command."""

    def __init__(self) -> None:
        super().__init__("xlerobot_leader_follower")
        self.declare_parameter("leader_topic", "leader/joint_states")
        self.declare_parameter("command_topic", "joint_commands")
        self.declare_parameter("source_joints", DEFAULT_JOINTS)
        self.declare_parameter("target_joints", DEFAULT_JOINTS)
        self.declare_parameter("scales", [1.0] * len(DEFAULT_JOINTS))
        self.declare_parameter("offsets", [0.0] * len(DEFAULT_JOINTS))
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("stale_timeout", 0.25)
        self.declare_parameter("filter_alpha", 0.35)
        self.declare_parameter("max_delta", 0.12)

        self._source_joints = list(self.get_parameter("source_joints").value)
        self._target_joints = list(self.get_parameter("target_joints").value)
        self._scales = list(self.get_parameter("scales").value)
        self._offsets = list(self.get_parameter("offsets").value)
        size = len(self._source_joints)
        if not size or not (
            len(self._target_joints) == len(self._scales) == len(self._offsets) == size
        ):
            raise ValueError("source_joints, target_joints, scales and offsets must match")

        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        stale_timeout = float(self.get_parameter("stale_timeout").value)
        filter_alpha = float(self.get_parameter("filter_alpha").value)
        max_delta = float(self.get_parameter("max_delta").value)
        if stale_timeout < 0.0:
            raise ValueError("stale_timeout must be non-negative")
        if not 0.0 < filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        if max_delta <= 0.0:
            raise ValueError("max_delta must be positive")

        leader_topic = str(self.get_parameter("leader_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        self._publisher = self.create_publisher(JointState, command_topic, 10)
        self.create_subscription(
            JointState, leader_topic, self._on_leader_state, qos_profile_sensor_data
        )
        self.create_timer(1.0 / rate, self._publish_command)

        self._raw_target: list[float] | None = None
        self._filtered: list[float] | None = None
        self._last_message_time = self.get_clock().now()
        self.get_logger().info(
            f"Relaying {size} joints from {leader_topic} to {command_topic}"
        )

    def _on_leader_state(self, msg: JointState) -> None:
        try:
            values = extract_positions(msg.name, msg.position, self._source_joints)
            self._raw_target = mapped_positions(values, self._scales, self._offsets)
            self._last_message_time = self.get_clock().now()
        except ValueError as exc:
            self.get_logger().warning(str(exc))

    def _publish_command(self) -> None:
        if self._raw_target is None:
            return
        age = (self.get_clock().now() - self._last_message_time).nanoseconds / 1e9
        if age > float(self.get_parameter("stale_timeout").value):
            return

        self._filtered = filtered_step(
            self._filtered,
            self._raw_target,
            float(self.get_parameter("filter_alpha").value),
            float(self.get_parameter("max_delta").value),
        )
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._target_joints
        msg.position = self._filtered
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LeaderFollowerTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
