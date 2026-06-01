import unittest

from rover_control.rover_kinematics_node import RoverController
import rclpy


class TestKinematics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RoverController()

    def tearDown(self):
        self.node.destroy_node()

    def test_stop(self):
        """Stop should result in zero angles and speeds."""
        angles, speeds = self.node.compute_wheel_commands(0.0, 0.0, 0.0)

        self.assertTrue(all(a == 0.0 for a in angles))
        self.assertTrue(all(s == 0.0 for s in speeds))

    def test_straight_motion(self):
        """Moving straight forward should result in zero angles and positive speeds."""
        vx = 1.0  # m/s
        angles, speeds = self.node.compute_wheel_commands(vx, 0.0, 0.0)

        self.assertTrue(all(a == 0.0 for a in angles))
        self.assertTrue(all(s > 0 for s in speeds))

    def test_spin_in_place(self):
        """Spinning result: symmetric angles and opposite speeds on left vs right."""
        vtheta = 1.0  # rad/s
        angles, speeds = self.node.compute_wheel_commands(0.0, 0.0, vtheta)

        self.assertAlmostEqual(angles[0], -angles[1], places=3)
        self.assertAlmostEqual(speeds[0], -speeds[1], places=3)

    def test_steering_limits(self):
        """Steering limits should be enforced."""
        vtheta = 1.0  # rad/s
        angles, _ = self.node.compute_wheel_commands(0.5, 0.0, vtheta)

        max_angle = max(abs(a) for a in angles)

        self.assertLessEqual(max_angle, self.node.max_steer)
