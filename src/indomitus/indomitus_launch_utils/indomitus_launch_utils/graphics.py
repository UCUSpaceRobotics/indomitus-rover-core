"""Environment defaults that keep Gazebo and RViz GUIs working across machines.

Handles X11 and Wayland sessions, native installs and containers
(Docker/Distrobox), and Intel/AMD/NVIDIA GPUs. Call the matching function at
the top of ``generate_launch_description()``; every process started by the
launch inherits the resulting environment.
"""
import glob
import os

NVIDIA_EGL_VENDOR_PATH = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
NVIDIA_GPUS_DIR = "/proc/driver/nvidia/gpus"


def is_wayland_session() -> bool:
    return (os.environ.get("XDG_SESSION_TYPE") == "wayland"
            or bool(os.environ.get("WAYLAND_DISPLAY")))


def has_nvidia_gpu() -> bool:
    """True only when the NVIDIA kernel driver actually drives a GPU visible here.

    The NVIDIA vendor JSON ships in the ZED-based images and /dev/nvidiactl can
    exist without a usable GPU (e.g. AMD/Intel-only laptops), so neither is
    enough on its own.
    """
    try:
        driver_reports_gpu = bool(os.listdir(NVIDIA_GPUS_DIR))
    except OSError:
        driver_reports_gpu = False
    return driver_reports_gpu and bool(glob.glob("/dev/nvidia[0-9]*"))


def configure_gui_environment() -> None:
    """Graphics defaults for any Qt + OGRE GUI (Gazebo, RViz)."""

    # Wayland GUI Crash Workaround. Gazebo and RViz render through GLX, which Qt
    # only provides on X11, so run them through XWayland. Gazebo otherwise
    # segfaults in QOpenGLContext::doneCurrent() and RViz fails with
    # "Invalid parentWindowHandle". Non-Wayland platforms a user picked
    # explicitly (e.g. offscreen for headless runs) are left alone.
    qpa_platform = os.environ.get("QT_QPA_PLATFORM", "")
    platform_is_default = not qpa_platform or "wayland" in qpa_platform
    if is_wayland_session() and os.environ.get("DISPLAY") and platform_is_default:
        os.environ["QT_QPA_PLATFORM"] = "xcb"

    # Nvidia Optimus & EGL Deadlock Workaround (Prevents EGL dri2 crash). Forcing
    # the NVIDIA vendor without a real NVIDIA GPU leaves Qt without a GL context.
    if os.path.exists(NVIDIA_EGL_VENDOR_PATH) and has_nvidia_gpu():
        os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
        os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")
        os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", NVIDIA_EGL_VENDOR_PATH)


def configure_gazebo_environment() -> None:
    """GUI defaults plus Gazebo transport settings for a local simulation."""
    configure_gui_environment()

    # IPC / Networking Fix (Forces local ZeroMQ. Prevents GUI black screen & spawner deaf loops).
    # GZ_* is read by Gazebo Harmonic+, IGN_* by Ignition Fortress.
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    os.environ.setdefault("IGN_IP", "127.0.0.1")
