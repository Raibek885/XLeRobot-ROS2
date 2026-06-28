"""ROS 2 node bridging standard messages to the existing XLeRobot driver."""

from importlib import import_module
from math import radians
from typing import Any

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory

from .conversions import joint_message_to_action, observation_to_joint_positions, twist_to_action
from .mock_robot import MockRobot


ROBOT_TYPES = {
    "xlerobot": (
        "xlerobot_driver.hardware.xlerobot",
        "XLerobotConfig",
        "XLerobot",
    ),
    "xlerobot_2wheels": (
        "xlerobot_driver.hardware.xlerobot_2wheels",
        "XLerobot2WheelsConfig",
        "XLerobot2Wheels",
    ),
    "xlerobot_mecanum": (
        "xlerobot_driver.hardware.xlerobot_mecanum",
        "XLerobotConfig",
        "XLerobot",
    ),
}


class XLeRobotDriver(Node):
    """Own the serial hardware and expose safe ROS command/state topics."""

    def __init__(self) -> None:
        super().__init__("xlerobot_driver")

        self.declare_parameter("robot_variant", "xlerobot")
        self.declare_parameter("robot_id", "xlerobot_ros2")
        self.declare_parameter("port1", "/dev/ttyACM0")
        self.declare_parameter("port2", "/dev/ttyACM1")
        self.declare_parameter("mock_hardware", False)
        self.declare_parameter("calibrate_on_connect", False)
        self.declare_parameter("disable_torque_on_disconnect", True)
        self.declare_parameter("arm_p_coefficient", 16)
        self.declare_parameter("max_relative_target_degrees", 15.0)
        self.declare_parameter("state_publish_rate", 15.0)
        self.declare_parameter("command_rate", 30.0)
        self.declare_parameter("cmd_vel_timeout", 0.25)
        self.declare_parameter("max_linear_speed", 0.30)
        self.declare_parameter("max_angular_speed", 1.57)
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("joint_command_topic", "joint_commands")
        self.declare_parameter("trajectory_command_topic", "joint_trajectory")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("base_velocity_topic", "base_velocity")

        arm_p_coefficient = int(self.get_parameter("arm_p_coefficient").value)
        if not 0 <= arm_p_coefficient <= 254:
            raise ValueError("arm_p_coefficient must be between 0 and 254")

        self._enabled = True
        self._closed = False
        self._joint_action: dict[str, float] = {}
        self._joint_dirty = False
        self._last_hardware_positions: dict[str, float] = {}
        self._last_cmd_vel: Twist | None = None
        self._last_cmd_vel_time = self.get_clock().now()
        self._base_timeout_stop_sent = True

        self._robot = self._make_robot()
        calibrate = self.get_parameter("calibrate_on_connect").value
        self._robot.connect(calibrate=bool(calibrate))

        joint_states_topic = self.get_parameter("joint_states_topic").value
        joint_command_topic = self.get_parameter("joint_command_topic").value
        trajectory_topic = self.get_parameter("trajectory_command_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        base_velocity_topic = self.get_parameter("base_velocity_topic").value

        self._joint_state_pub = self.create_publisher(
            JointState, joint_states_topic, qos_profile_sensor_data
        )
        self._base_velocity_pub = self.create_publisher(
            TwistStamped, base_velocity_topic, qos_profile_sensor_data
        )
        self.create_subscription(JointState, joint_command_topic, self._on_joint_command, 10)
        self.create_subscription(JointTrajectory, trajectory_topic, self._on_trajectory, 10)
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_service(SetBool, "enable", self._on_enable)

        state_rate = float(self.get_parameter("state_publish_rate").value)
        command_rate = float(self.get_parameter("command_rate").value)
        if state_rate <= 0.0 or command_rate <= 0.0:
            raise ValueError("state_publish_rate and command_rate must be positive")
        if float(self.get_parameter("cmd_vel_timeout").value) < 0.0:
            raise ValueError("cmd_vel_timeout must be non-negative")
        if float(self.get_parameter("max_linear_speed").value) <= 0.0:
            raise ValueError("max_linear_speed must be positive")
        if float(self.get_parameter("max_angular_speed").value) <= 0.0:
            raise ValueError("max_angular_speed must be positive")
        if float(self.get_parameter("max_relative_target_degrees").value) < 0.0:
            raise ValueError("max_relative_target_degrees must be non-negative")
        self.create_timer(1.0 / state_rate, self._publish_state)
        self.create_timer(1.0 / command_rate, self._send_commands)

        mode = "mock" if self.get_parameter("mock_hardware").value else "real hardware"
        self.get_logger().info(f"XLeRobot driver connected in {mode} mode")

    def _make_robot(self) -> Any:
        if self.get_parameter("mock_hardware").value:
            return MockRobot()

        variant = str(self.get_parameter("robot_variant").value)
        try:
            module_name, config_name, robot_name = ROBOT_TYPES[variant]
        except KeyError as exc:
            supported = ", ".join(sorted(ROBOT_TYPES))
            raise ValueError(
                f"Unsupported robot_variant '{variant}'. Choose: {supported}"
            ) from exc

        try:
            module = import_module(module_name)
        except ImportError as exc:
            raise RuntimeError(
                f"Could not import {module_name}. Install LeRobot with Feetech support "
                "in the Python environment used to build this ROS workspace."
            ) from exc

        config_class = getattr(module, config_name)
        robot_class = getattr(module, robot_name)
        kwargs = {
            "id": str(self.get_parameter("robot_id").value),
            "port1": str(self.get_parameter("port1").value),
            "port2": str(self.get_parameter("port2").value),
            "use_degrees": True,
            "arm_p_coefficient": int(
                self.get_parameter("arm_p_coefficient").value
            ),
            "disable_torque_on_disconnect": bool(
                self.get_parameter("disable_torque_on_disconnect").value
            ),
            # The ROS bridge applies this bound itself, which also works across
            # the XLeRobot driver versions currently present in this repository.
            "max_relative_target": None,
        }
        return robot_class(config_class(**kwargs))

    def _on_joint_command(self, msg: JointState) -> None:
        try:
            self._joint_action = joint_message_to_action(msg.name, msg.position)
            self._joint_dirty = True
        except ValueError as exc:
            self.get_logger().error(str(exc))

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warning("Ignoring JointTrajectory without points")
            return
        try:
            self._joint_action = joint_message_to_action(
                msg.joint_names, msg.points[-1].positions
            )
            self._joint_dirty = True
        except ValueError as exc:
            self.get_logger().error(str(exc))

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel = msg
        self._last_cmd_vel_time = self.get_clock().now()
        self._base_timeout_stop_sent = False

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._enabled = bool(request.data)
        if not self._enabled:
            self._joint_dirty = False
            self._stop_base()
        response.success = True
        response.message = (
            "Commands enabled" if self._enabled else "Commands disabled; base stopped"
        )
        return response

    def _send_commands(self) -> None:
        if not self._enabled:
            return

        action: dict[str, float] = {}
        if self._joint_dirty:
            action.update(self._bounded_joint_action(self._joint_action))
            self._joint_dirty = False

        if self._last_cmd_vel is not None:
            age = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9
            timeout = float(self.get_parameter("cmd_vel_timeout").value)
            if age <= timeout:
                msg = self._last_cmd_vel
                try:
                    max_linear = float(self.get_parameter("max_linear_speed").value)
                    max_angular = float(self.get_parameter("max_angular_speed").value)
                    linear_x = min(max_linear, max(-max_linear, msg.linear.x))
                    linear_y = min(max_linear, max(-max_linear, msg.linear.y))
                    angular_z = min(max_angular, max(-max_angular, msg.angular.z))
                    action.update(
                        twist_to_action(linear_x, linear_y, angular_z)
                    )
                except ValueError as exc:
                    self.get_logger().error(str(exc))
                self._base_timeout_stop_sent = False
            elif not self._base_timeout_stop_sent:
                action.update(twist_to_action(0.0, 0.0, 0.0))
                self._base_timeout_stop_sent = True

        if not action:
            return
        try:
            self._robot.send_action(action)
        except Exception as exc:  # Hardware errors should not kill the ROS graph.
            self.get_logger().error(f"Failed to send robot command: {exc}")

    def _publish_state(self) -> None:
        try:
            observation = self._robot.get_observation()
            self._last_hardware_positions = {
                key: float(value)
                for key, value in observation.items()
                if key.endswith(".pos")
            }
            names, positions = observation_to_joint_positions(observation)
        except Exception as exc:
            self.get_logger().error(f"Failed to read robot state: {exc}")
            return

        stamp = self.get_clock().now().to_msg()
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = names
        joint_state.position = positions
        self._joint_state_pub.publish(joint_state)

        velocity = TwistStamped()
        velocity.header.stamp = stamp
        velocity.header.frame_id = "base_link"
        velocity.twist.linear.x = float(observation.get("x.vel", 0.0))
        velocity.twist.linear.y = float(observation.get("y.vel", 0.0))
        velocity.twist.angular.z = radians(float(observation.get("theta.vel", 0.0)))
        self._base_velocity_pub.publish(velocity)

    def _bounded_joint_action(self, action: dict[str, float]) -> dict[str, float]:
        max_delta = float(self.get_parameter("max_relative_target_degrees").value)
        if max_delta <= 0.0 or not self._last_hardware_positions:
            return dict(action)
        bounded: dict[str, float] = {}
        for key, target in action.items():
            current = self._last_hardware_positions.get(key)
            if current is None:
                bounded[key] = target
                continue
            bounded[key] = min(current + max_delta, max(current - max_delta, target))
        return bounded

    def _stop_base(self) -> None:
        try:
            if hasattr(self._robot, "stop_base"):
                self._robot.stop_base()
            else:
                self._robot.send_action(twist_to_action(0.0, 0.0, 0.0))
            self._base_timeout_stop_sent = True
        except Exception as exc:
            self.get_logger().error(f"Failed to stop base: {exc}")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_base()
        try:
            self._robot.disconnect()
        except Exception as exc:
            self.get_logger().error(f"Failed to disconnect robot cleanly: {exc}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: XLeRobotDriver | None = None
    try:
        node = XLeRobotDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
