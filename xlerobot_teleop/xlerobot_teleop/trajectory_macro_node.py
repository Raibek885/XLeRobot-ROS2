"""Record and replay a competition joint macro from the Xbox controller."""

from __future__ import annotations

from math import degrees
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy

from .trajectory_macro import JointMacro, interpolate_positions


DEFAULT_JOINTS = [
    "left_arm_shoulder_pan",
    "left_arm_shoulder_lift",
    "left_arm_elbow_flex",
    "left_arm_wrist_flex",
    "left_arm_wrist_roll",
    "right_arm_shoulder_pan",
    "right_arm_shoulder_lift",
    "right_arm_elbow_flex",
    "right_arm_wrist_flex",
    "right_arm_wrist_roll",
]

ACTIVE_REPLAY_MODES = {"transition", "settle", "replay"}


def _pressed(buttons: list[int], index: int) -> bool:
    return 0 <= index < len(buttons) and bool(buttons[index])


class TrajectoryMacroNode(Node):
    """Record actual arm states and replay them from a guarded start pose."""

    def __init__(self) -> None:
        super().__init__("xlerobot_trajectory_macro")
        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("joint_command_topic", "joint_commands")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter(
            "macro_path", "~/.ros/xlerobot_macros/connect_sticks.json"
        )
        self.declare_parameter("joint_names", DEFAULT_JOINTS)
        self.declare_parameter("sample_rate", 30.0)
        self.declare_parameter("record_button", 4)  # Y
        self.declare_parameter("record_modifier_button", 1)  # B
        self.declare_parameter("base_button", 0)  # A
        self.declare_parameter("left_arm_button", 6)  # LB
        self.declare_parameter("right_arm_button", 7)  # RB
        self.declare_parameter("head_button", 11)  # Start/Menu
        self.declare_parameter("transition_duration", 4.0)
        self.declare_parameter("settle_duration", 0.5)
        self.declare_parameter("max_start_delta_degrees", 35.0)
        self.declare_parameter("minimum_recording_duration", 0.5)
        self.declare_parameter("joy_timeout", 0.5)

        self._joint_names = [
            str(name) for name in self.get_parameter("joint_names").value
        ]
        self._sample_rate = float(self.get_parameter("sample_rate").value)
        if self._sample_rate <= 0.0:
            raise ValueError("sample_rate must be positive")
        if not self._joint_names or len(set(self._joint_names)) != len(
            self._joint_names
        ):
            raise ValueError("joint_names must be non-empty and unique")

        self._macro_path = Path(
            str(self.get_parameter("macro_path").value)
        ).expanduser()
        self._state: dict[str, float] = {}
        self._last_buttons: list[int] = []
        self._last_joy_time = self.get_clock().now()
        self._joy_seen = False
        self._mode = "idle"
        self._frames: list[list[float]] = []
        self._macro: JointMacro | None = None
        self._phase_start = self.get_clock().now()
        self._transition_start: list[float] = []

        self._joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_command_topic").value), 10
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Joy,
            str(self.get_parameter("joy_topic").value),
            self._on_joy,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / self._sample_rate, self._tick)

        self.get_logger().info(
            "Trajectory macro ready: B+Y starts/stops recording, Y replays, "
            "B or manual control cancels"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            self.get_logger().warning("Ignoring malformed JointState")
            return
        self._state.update(
            {name: float(position) for name, position in zip(msg.name, msg.position)}
        )

    def _positions(self) -> list[float] | None:
        missing = [name for name in self._joint_names if name not in self._state]
        if missing:
            self.get_logger().warning(
                "Cannot use trajectory macro; missing joint states: " + ", ".join(missing)
            )
            return None
        return [self._state[name] for name in self._joint_names]

    def _on_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        self._last_joy_time = self.get_clock().now()
        self._joy_seen = True

        record_button = int(self.get_parameter("record_button").value)
        modifier_button = int(self.get_parameter("record_modifier_button").value)
        record_pressed = _pressed(buttons, record_button)
        modifier_pressed = _pressed(buttons, modifier_button)
        previous_record = _pressed(self._last_buttons, record_button)
        previous_modifier = _pressed(self._last_buttons, modifier_button)
        record_chord_rising = (
            record_pressed
            and modifier_pressed
            and not (previous_record and previous_modifier)
        )
        replay_rising = record_pressed and not previous_record and not modifier_pressed

        if record_chord_rising:
            if self._mode == "recording":
                self._stop_recording()
            elif self._mode in ACTIVE_REPLAY_MODES:
                self._cancel_replay("recording chord pressed")
            else:
                self._start_recording()
        elif replay_rising:
            if self._mode in ACTIVE_REPLAY_MODES:
                self._cancel_replay("Y pressed during replay")
            elif self._mode == "recording":
                self.get_logger().warning("Use B+Y to finish and save the recording")
            else:
                self._start_replay()

        if self._mode in ACTIVE_REPLAY_MODES:
            manual_buttons = [
                int(self.get_parameter("base_button").value),
                int(self.get_parameter("left_arm_button").value),
                int(self.get_parameter("right_arm_button").value),
                int(self.get_parameter("head_button").value),
            ]
            manual_pressed = any(_pressed(buttons, index) for index in manual_buttons)
            cancel_pressed = modifier_pressed and not record_pressed
            if manual_pressed or cancel_pressed:
                self._cancel_replay("manual control requested")

        self._last_buttons = buttons

    def _start_recording(self) -> None:
        positions = self._positions()
        if positions is None:
            return
        self._frames = [positions]
        self._mode = "recording"
        self._phase_start = self.get_clock().now()
        self.get_logger().info(
            "RECORDING started. This first frame is the task_ready pose. "
            "Press B+Y again to save."
        )

    def _stop_recording(self) -> None:
        minimum_duration = float(
            self.get_parameter("minimum_recording_duration").value
        )
        duration = max(0.0, (len(self._frames) - 1) / self._sample_rate)
        if duration < minimum_duration:
            self.get_logger().error(
                f"Recording is only {duration:.2f}s; keeping the previous macro"
            )
            self._mode = "idle"
            self._frames = []
            return
        try:
            macro = JointMacro.from_frames(
                self._joint_names, self._sample_rate, self._frames
            )
            macro.save(self._macro_path)
        except Exception as exc:
            self.get_logger().error(f"Failed to save trajectory macro: {exc}")
        else:
            self._macro = macro
            self.get_logger().info(
                f"SAVED {len(macro.frames)} frames ({macro.duration:.2f}s) to "
                f"{self._macro_path}"
            )
        finally:
            self._mode = "idle"
            self._frames = []

    def _start_replay(self) -> None:
        positions = self._positions()
        if positions is None:
            return
        try:
            macro = JointMacro.load(self._macro_path)
        except FileNotFoundError:
            self.get_logger().error(
                f"No macro found at {self._macro_path}. Record one with B+Y first."
            )
            return
        except Exception as exc:
            self.get_logger().error(f"Failed to load trajectory macro: {exc}")
            return
        if macro.joint_names != self._joint_names:
            self.get_logger().error("Recorded macro joint list does not match this robot")
            return

        max_delta = max(
            abs(goal - current)
            for current, goal in zip(positions, macro.frames[0])
        )
        max_allowed = float(
            self.get_parameter("max_start_delta_degrees").value
        )
        if degrees(max_delta) > max_allowed:
            self.get_logger().error(
                f"Refusing replay: current pose is {degrees(max_delta):.1f} deg from "
                f"task_ready (limit {max_allowed:.1f} deg). Move closer manually."
            )
            return

        self._macro = macro
        self._transition_start = positions
        self._phase_start = self.get_clock().now()
        self._mode = "transition"
        self.get_logger().info(
            "REPLAY: moving both arms to task_ready. Press B, A, LB, RB or Start "
            "to cancel."
        )

    def _cancel_replay(self, reason: str) -> None:
        if self._mode not in ACTIVE_REPLAY_MODES:
            return
        self._mode = "idle"
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().warning(f"REPLAY cancelled: {reason}")

    def _publish_positions(self, positions: list[float]) -> None:
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = list(self._joint_names)
        command.position = list(positions)
        self._joint_pub.publish(command)
        self._cmd_vel_pub.publish(Twist())

    def _tick(self) -> None:
        now = self.get_clock().now()
        if self._mode in ACTIVE_REPLAY_MODES:
            joy_age = (now - self._last_joy_time).nanoseconds / 1e9
            if not self._joy_seen or joy_age > float(
                self.get_parameter("joy_timeout").value
            ):
                self._cancel_replay("gamepad connection lost")
                return

        if self._mode == "recording":
            positions = self._positions()
            if positions is not None:
                self._frames.append(positions)
            return

        if self._macro is None or self._mode == "idle":
            return

        elapsed = (now - self._phase_start).nanoseconds / 1e9
        if self._mode == "transition":
            duration = max(
                0.1, float(self.get_parameter("transition_duration").value)
            )
            self._publish_positions(
                interpolate_positions(
                    self._transition_start, self._macro.frames[0], elapsed / duration
                )
            )
            if elapsed >= duration:
                self._mode = "settle"
                self._phase_start = now
                self.get_logger().info("REPLAY: task_ready reached")
            return

        if self._mode == "settle":
            self._publish_positions(self._macro.frames[0])
            if elapsed >= max(
                0.0, float(self.get_parameter("settle_duration").value)
            ):
                self._mode = "replay"
                self._phase_start = now
                self.get_logger().info("REPLAY: executing recorded connection")
            return

        frame_index = int(elapsed * self._macro.sample_rate)
        if frame_index >= len(self._macro.frames):
            self._publish_positions(self._macro.frames[-1])
            self._mode = "idle"
            self.get_logger().info("REPLAY completed")
            return
        self._publish_positions(self._macro.frames[frame_index])


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryMacroNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
