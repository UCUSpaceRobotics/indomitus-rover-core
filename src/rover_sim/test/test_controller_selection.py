"""
Which swerve controller comes up active in simulation.

RoverSwerveControllerTest is experimental. It is spawned so it can be switched
to at runtime, but it must not be what the rover starts driving on unless
somebody asked for it — that is a one-word edit in a launch file, and exactly
the kind of change that survives review by being invisible.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory

import pytest


def _load_launch_module():
    path = os.path.join(
        get_package_share_directory('rover_sim'), 'launch', 'sim_gz.launch.py')
    spec = importlib.util.spec_from_file_location('sim_gz_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def launch_module():
    return _load_launch_module()


def test_default_is_the_production_controller(launch_module):
    assert launch_module.DEFAULT_SWERVE_CONTROLLER == 'swerve_controller'

    controllers, inactive = launch_module.resolve_controllers(
        launch_module.DEFAULT_SWERVE_CONTROLLER)

    assert 'swerve_controller' in controllers.split()
    assert 'swerve_controller_test' in inactive.split(), \
        'the experimental controller must be spawned inactive by default'
    assert 'swerve_controller' not in inactive.split()


def test_experimental_controller_can_be_opted_into(launch_module):
    controllers, inactive = launch_module.resolve_controllers(
        launch_module.EXPERIMENTAL_SWERVE_CONTROLLER)

    assert 'swerve_controller_test' in controllers.split()
    assert inactive.split() == ['swerve_controller'], \
        'only one controller may hold the joints at a time'


def test_both_controllers_are_always_spawned(launch_module):
    for choice in launch_module.SWERVE_CONTROLLERS:
        controllers, _ = launch_module.resolve_controllers(choice)
        names = controllers.split()
        assert 'swerve_controller' in names
        assert 'swerve_controller_test' in names
        assert len(names) == len(set(names)), f'duplicate spawner in {names}'


def test_unknown_controller_is_rejected(launch_module):
    with pytest.raises(RuntimeError):
        launch_module.resolve_controllers('odometry_controller')
