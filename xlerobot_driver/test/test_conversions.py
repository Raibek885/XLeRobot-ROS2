from math import isclose, pi
import unittest

from xlerobot_driver.conversions import (
    hardware_to_ros_position,
    joint_message_to_action,
    observation_to_joint_positions,
    ros_to_hardware_position,
    twist_to_action,
)


class ConversionTests(unittest.TestCase):
    def test_revolute_joint_units_round_trip(self):
        hardware = ros_to_hardware_position("left_arm_shoulder_pan", pi / 2.0)
        self.assertTrue(isclose(hardware, 90.0))
        self.assertTrue(
            isclose(hardware_to_ros_position("left_arm_shoulder_pan", hardware), pi / 2.0)
        )

    def test_gripper_is_normalized_and_clamped(self):
        self.assertEqual(ros_to_hardware_position("left_arm_gripper", 0.25), 25.0)
        self.assertEqual(ros_to_hardware_position("left_arm_gripper", 2.0), 100.0)
        self.assertEqual(hardware_to_ros_position("right_arm_gripper", -10.0), 0.0)

    def test_partial_joint_command(self):
        action = joint_message_to_action(
            ["head_motor_1", "left_arm_gripper"], [pi, 0.4]
        )
        self.assertTrue(isclose(action["head_motor_1.pos"], 180.0))
        self.assertEqual(action["left_arm_gripper.pos"], 40.0)

    def test_invalid_joint_command_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unknown"):
            joint_message_to_action(["not_a_joint"], [0.0])
        with self.assertRaisesRegex(ValueError, "equal length"):
            joint_message_to_action(["head_motor_1"], [])
        with self.assertRaisesRegex(ValueError, "duplicate"):
            joint_message_to_action(["head_motor_1", "head_motor_1"], [0.0, 0.1])

    def test_observation_order_and_twist_conversion(self):
        names, positions = observation_to_joint_positions(
            {"head_motor_2.pos": 90.0, "left_arm_gripper.pos": 50.0}
        )
        self.assertEqual(names, ["left_arm_gripper", "head_motor_2"])
        self.assertTrue(isclose(positions[0], 0.5))
        self.assertTrue(isclose(positions[1], pi / 2.0))
        action = twist_to_action(0.2, -0.1, pi)
        self.assertEqual(
            action, {"x.vel": 0.2, "y.vel": -0.1, "theta.vel": 180.0}
        )
