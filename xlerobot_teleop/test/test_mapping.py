import unittest

from xlerobot_teleop.mapping import (
    axis_value,
    button_pressed,
    extract_positions,
    filtered_step,
    mapped_positions,
)


class MappingTests(unittest.TestCase):
    def test_safe_joy_access_and_deadzone(self):
        self.assertEqual(axis_value([0.05, 1.0], 0, 0.1), 0.0)
        self.assertEqual(axis_value([0.05, 1.0], 1, 0.1), 1.0)
        self.assertEqual(axis_value([], 8, 0.1), 0.0)
        self.assertTrue(button_pressed([0, 1], 1))
        self.assertFalse(button_pressed([], 1))
        with self.assertRaisesRegex(ValueError, "deadzone"):
            axis_value([1.0], 0, 1.0)

    def test_extract_positions_is_name_based(self):
        values = extract_positions(["b", "a"], [2.0, 1.0], ["a", "b"])
        self.assertEqual(values, [1.0, 2.0])
        with self.assertRaisesRegex(ValueError, "Missing"):
            extract_positions(["a"], [1.0], ["a", "b"])

    def test_filter_and_step_limit(self):
        self.assertEqual(filtered_step(None, [1.0], 0.5, 0.1), [1.0])
        self.assertEqual(filtered_step([0.0], [1.0], 0.5, 0.1), [0.1])
        self.assertEqual(filtered_step([0.0], [-1.0], 1.0, 0.2), [-0.2])

    def test_affine_joint_mapping(self):
        self.assertEqual(
            mapped_positions([1.0, 2.0], [-1.0, 0.5], [0.2, 0.0]),
            [-0.8, 1.0],
        )
