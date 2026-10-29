import importlib.util
import os

import pytest

pytest.importorskip('launch_ros')

_PATH = os.path.join(os.path.dirname(__file__), '..', 'launch', 'cameras.launch.py')
_spec = importlib.util.spec_from_file_location('cameras_launch', _PATH)
cameras_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cameras_launch)
select = cameras_launch.select_cameras

CAMERAS = {'mast': {}, 'arm': None, 'zed': {'autostart': False}}


def test_default_starts_all_with_autostart():
    assert select(CAMERAS, {}, [], [], []) == ['mast', 'arm']


def test_autostart_falls_back_to_defaults():
    assert select(CAMERAS, {'autostart': False}, [], [], []) == []
    assert select({'a': {'autostart': True}}, {'autostart': False}, [], [], []) == ['a']


def test_named_cameras_start_regardless_of_autostart_in_file_order():
    assert select(CAMERAS, {}, ['zed', 'mast'], [], []) == ['mast', 'zed']


def test_exclude_drops_from_either_selection():
    assert select(CAMERAS, {}, [], [], ['arm']) == ['mast']
    assert select(CAMERAS, {}, ['mast', 'zed'], [], ['zed']) == ['mast']


def test_all_starts_every_camera():
    assert select(CAMERAS, {}, ['all'], [], []) == ['mast', 'arm', 'zed']
    assert select(CAMERAS, {}, ['all'], [], ['arm']) == ['mast', 'zed']


def test_include_adds_to_the_default_set():
    assert select(CAMERAS, {}, [], ['zed'], []) == ['mast', 'arm', 'zed']
    assert select(CAMERAS, {}, [], ['zed'], ['arm']) == ['mast', 'zed']
    assert select(CAMERAS, {}, ['arm'], ['zed'], []) == ['arm', 'zed']


def test_exclude_wins_over_include():
    assert select(CAMERAS, {}, [], ['zed'], ['zed']) == ['mast', 'arm']


@pytest.mark.parametrize('wanted, included, excluded', [
    (['mats'], [], []), ([], ['zde'], []), ([], [], ['amr']), (['all', 'zed'], [], [])])
def test_unknown_names_are_an_error(wanted, included, excluded):
    with pytest.raises(RuntimeError, match='no such camera'):
        select(CAMERAS, {}, wanted, included, excluded)


def test_a_camera_called_all_is_an_error():
    with pytest.raises(RuntimeError, match="'all'"):
        select({'all': {}}, {}, [], [], [])


def test_names_splits_and_trims():
    assert cameras_launch._names(' a, b,,c ') == ['a', 'b', 'c']
    assert cameras_launch._names('') == []
