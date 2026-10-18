import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'examples/ros2_tools'))
from control_math import absolute_command


class ControlMathTest(unittest.TestCase):
    def test_right_target_decreases_yaw(self):
        self.assertEqual(absolute_command(10, 0, 1, 0), (9, 0))

    def test_up_target_decreases_pitch(self):
        self.assertEqual(absolute_command(0, 10, 0, 1), (0, 9))

    def test_limits_large_error(self):
        self.assertEqual(absolute_command(0, 0, 30, -30), (-2, 2))

    def test_deadband(self):
        self.assertEqual(absolute_command(10, 5, 0.1, -0.1), (10, 5))

    def test_yaw_wrap(self):
        self.assertEqual(absolute_command(-179, 0, 2, 0), (179, 0))

    def test_invalid_input(self):
        for value in [float('nan'), float('inf')]:
            with self.assertRaises(ValueError):
                absolute_command(0, 0, value, 0)

    def test_invalid_parameters(self):
        with self.assertRaises(ValueError):
            absolute_command(0, 0, 0, 0, max_step=0)


if __name__ == '__main__':
    unittest.main()
