import os

import pytest

from indomitus_launch_utils import graphics

MANAGED_VARS = (
    "XDG_SESSION_TYPE", "WAYLAND_DISPLAY", "DISPLAY", "QT_QPA_PLATFORM",
    "__NV_PRIME_RENDER_OFFLOAD", "__GLX_VENDOR_LIBRARY_NAME", "__EGL_VENDOR_LIBRARY_FILENAMES",
    "GZ_IP", "IGN_IP",
)
real_has_nvidia_gpu = graphics.has_nvidia_gpu


@pytest.fixture(autouse=True)
def clean_env(monkeypatch):
    for name in MANAGED_VARS:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setattr(graphics, "has_nvidia_gpu", lambda: False)


def test_x11_session_is_untouched(monkeypatch):
    monkeypatch.setenv("XDG_SESSION_TYPE", "x11")
    monkeypatch.setenv("DISPLAY", ":0")
    graphics.configure_gui_environment()
    assert "QT_QPA_PLATFORM" not in os.environ


@pytest.mark.parametrize("session_vars", [
    {"XDG_SESSION_TYPE": "wayland"},
    {"WAYLAND_DISPLAY": "wayland-0"},  # e.g. containers that drop XDG_SESSION_TYPE
])
def test_wayland_uses_xwayland(monkeypatch, session_vars):
    for name, value in session_vars.items():
        monkeypatch.setenv(name, value)
    monkeypatch.setenv("DISPLAY", ":0")
    graphics.configure_gui_environment()
    assert os.environ["QT_QPA_PLATFORM"] == "xcb"


@pytest.mark.parametrize("platform", ["wayland", "wayland;xcb", "wayland-egl"])
def test_wayland_platform_from_session_is_overridden(monkeypatch, platform):
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    monkeypatch.setenv("DISPLAY", ":0")
    monkeypatch.setenv("QT_QPA_PLATFORM", platform)
    graphics.configure_gui_environment()
    assert os.environ["QT_QPA_PLATFORM"] == "xcb"


def test_explicit_non_wayland_platform_is_kept(monkeypatch):
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    monkeypatch.setenv("DISPLAY", ":0")
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    graphics.configure_gui_environment()
    assert os.environ["QT_QPA_PLATFORM"] == "offscreen"


def test_wayland_without_xwayland_is_untouched(monkeypatch):
    monkeypatch.setenv("WAYLAND_DISPLAY", "wayland-0")
    graphics.configure_gui_environment()
    assert "QT_QPA_PLATFORM" not in os.environ


def test_nvidia_vendor_requires_real_gpu(monkeypatch):
    monkeypatch.setattr(graphics.os.path, "exists", lambda path: True)
    graphics.configure_gui_environment()
    assert "__EGL_VENDOR_LIBRARY_FILENAMES" not in os.environ

    monkeypatch.setattr(graphics, "has_nvidia_gpu", lambda: True)
    graphics.configure_gui_environment()
    assert os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] == graphics.NVIDIA_EGL_VENDOR_PATH
    assert os.environ["__GLX_VENDOR_LIBRARY_NAME"] == "nvidia"


def test_has_nvidia_gpu_ignores_stale_control_node(monkeypatch):
    # /dev/nvidiactl alone (no GPU reported by the driver) is not an NVIDIA GPU.
    def no_driver(path):
        raise FileNotFoundError(path)

    monkeypatch.setattr(graphics.os, "listdir", no_driver)
    monkeypatch.setattr(graphics.glob, "glob", lambda pattern: [])
    assert real_has_nvidia_gpu() is False

    monkeypatch.setattr(graphics.os, "listdir", lambda path: ["0000:01:00.0"])
    assert real_has_nvidia_gpu() is False

    monkeypatch.setattr(graphics.glob, "glob", lambda pattern: ["/dev/nvidia0"])
    assert real_has_nvidia_gpu() is True


def test_gazebo_transport_defaults_respect_overrides(monkeypatch):
    monkeypatch.setenv("GZ_IP", "10.0.0.5")
    graphics.configure_gazebo_environment()
    assert os.environ["GZ_IP"] == "10.0.0.5"
    assert os.environ["IGN_IP"] == "127.0.0.1"
