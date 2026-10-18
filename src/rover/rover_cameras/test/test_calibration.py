from rover_cameras import calibration

CALIB = """\
image_width: 960
image_height: 600
camera_name: arducam-front
camera_matrix:
  rows: 3
  cols: 3
  data: [800.0, 0.0, 480.0, 0.0, 810.0, 300.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.1, 0.02, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [790.0, 0.0, 480.0, 0.0, 0.0, 805.0, 300.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


def test_load_and_scale(tmp_path):
    path = tmp_path / 'c.yaml'
    path.write_text(CALIB)
    info = calibration.load(str(path))
    assert info['k'][0] == 800.0 and info['d'][0] == -0.1

    half = calibration.scaled(info, 480, 300)
    assert half['k'][:6] == [400.0, 0.0, 240.0, 0.0, 405.0, 150.0]
    assert half['p'][0] == 395.0 and half['p'][6] == 150.0
    assert half['d'] == info['d']
    assert calibration.scaled(info, 960, 600) == info
