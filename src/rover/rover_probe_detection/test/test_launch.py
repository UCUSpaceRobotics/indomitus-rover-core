import importlib.util
import os

import pytest

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'probe_detection.launch.py')


def test_parse_probes():
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location('probe_launch', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.parse_probes('') == []
    assert mod.parse_probes('1,2,0; 3,-1,0.5') == [1.0, 2.0, 0.0, 3.0, -1.0, 0.5]
    with pytest.raises(RuntimeError):
        mod.parse_probes('1,2')
