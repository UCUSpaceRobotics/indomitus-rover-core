import importlib.util
import os

import pytest

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'cameras.launch.py')


@pytest.fixture(scope='module')
def launch():
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location('cameras_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_defaults_and_overrides(launch, tmp_path):
    ros_name, p = launch.camera_params(
        'arducam-front', {'device': '/dev/arducam-front', 'port': 8092, 'ros_res': '480x300'},
        {'res': '960x600', 'fps': 15, 'quality': 60}, str(tmp_path))
    assert ros_name == 'arducam_front'
    assert (p['width'], p['height'], p['ros_width'], p['ros_height']) == (960, 600, 480, 300)
    assert p['fps'] == 15.0 and p['quality'] == 60 and p['port'] == 8092
    assert p['frame_id'] == 'arducam_front_optical_frame'
    assert 'camera_info_file' not in p


def test_calibration_found_by_name(launch, tmp_path):
    (tmp_path / 'cam.yaml').write_text('x')
    _, p = launch.camera_params('cam', {'device': '/dev/x'}, {}, str(tmp_path))
    assert p['camera_info_file'] == str(tmp_path / 'cam.yaml')


def test_named_calibration_must_exist(launch, tmp_path):
    with pytest.raises(RuntimeError):
        launch.camera_params('cam', {'device': '/dev/x', 'calibration': 'nope.yaml'}, {},
                             str(tmp_path))
