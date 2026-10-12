"""Shared rclpy lifecycle for this package's tests.

Session-scoped (not per-module): two test files here now each need a live
rclpy context (test_servo_teleop.py, test_arm_motion_lock_server.py)
— two independent module-scoped init/shutdown fixtures race each other's
teardown when pytest runs both in one session.
"""
import rclpy
import pytest


@pytest.fixture(scope='session', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()
