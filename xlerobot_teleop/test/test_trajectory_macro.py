import tempfile
import unittest
from pathlib import Path

from xlerobot_teleop.trajectory_macro import JointMacro, interpolate_positions


class TrajectoryMacroTests(unittest.TestCase):
    def test_round_trip(self):
        macro = JointMacro.from_frames(
            ["left", "right"], 10.0, [[0.0, 1.0], [0.5, 1.5], [1.0, 2.0]]
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "macro.json"
            macro.save(path)
            loaded = JointMacro.load(path)
        self.assertEqual(loaded.joint_names, macro.joint_names)
        self.assertEqual(loaded.frames, macro.frames)
        self.assertEqual(loaded.duration, 0.2)

    def test_interpolation_is_clamped(self):
        self.assertEqual(interpolate_positions([0.0], [2.0], -1.0), [0.0])
        self.assertEqual(interpolate_positions([0.0], [2.0], 0.25), [0.5])
        self.assertEqual(interpolate_positions([0.0], [2.0], 2.0), [2.0])

    def test_rejects_malformed_frames(self):
        with self.assertRaisesRegex(ValueError, "expected"):
            JointMacro.from_frames(["a", "b"], 30.0, [[1.0]])

