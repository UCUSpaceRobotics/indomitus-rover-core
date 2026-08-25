"""Drives the arm's real OV5693 camera through gscam, in one of two
GStreamer pipelines selected by the "backend" launch argument:

  backend:=usb (default) - v4l2src against a USB/UVC bridge board, for
    testing on a regular dev machine. Confirmed live against the actual
    OV5693 (shows up as a generic "USB Camera", vendor 0ede, driver
    uvcvideo), reached via its udev-stable by-id path rather than the
    raw /dev/videoN (see video_device below -- that node number isn't
    stable across reboots/replugs/other USB devices, the by-id symlink
    is). Captures raw YUYV, not this camera's MJPEG mode -- confirmed
    live that MJPEG decoding (jpegdec) intermittently corrupts frames
    ("sequence size exceeds remaining buffer", occasionally fatal) on
    this specific camera/USB stack, while YUYV never once did across
    repeated sustained tests. The real tradeoff is resolution/framerate,
    not a settings knob: this camera's YUYV mode only hits ~29fps at
    640x480 (this backend's default) -- at 1280x720 YUYV caps at
    10fps by the camera's own spec, and came out slower than that
    (~5-6fps) in practice, almost certainly USB2.0 bandwidth (raw YUYV
    at 720p is ~4x a same-resolution MJPEG stream). Bump width/height
    only with that ceiling in mind.

  backend:=csi - EXPERIMENTAL, UNVERIFIED. nvarguscamerasrc against a
    native MIPI CSI-2 connection on a Jetson (the eventual real
    deployment target), but this whole code path has only ever been
    reviewed, never run: nvarguscamerasrc is a Jetson-only GStreamer
    element (confirmed absent on this x86 dev machine via
    `gst-inspect-1.0`), so there is no way to test any of this off a
    real Jetson. Do not trust this on the real arm until it's actually
    been run and verified there -- `usb` is the only backend anything
    here has confirmed working. Defaults to 1280x720 (a practical
    resolution for the CV pipeline, not a hardware limit) rather than
    the OV5693's native 2592x1944 -- csi_width/csi_height can still
    request the full 5MP when actually needed, but the exact supported
    resolution/framerate combinations still need verifying on real
    hardware.

Publishes to /camera/image_raw + /camera/camera_info with
frame_id=arm_camera_optical_frame by default, matching exactly what the
simulated wrist camera already publishes (see arm_sim's
arm_gazebo.launch.py + config/gz_bridge.yaml) -- aruco_tracker,
panel_pose_fuser_node, and everything else downstream in the CV pipeline
need no changes to run against this real driver instead of the sim one.

camera_info_url defaults to this package's own checked-in calibration
(config/ov5693_usb.yaml, done with the wrist camera mounted on the arm)
-- if a DIFFERENT physical camera unit or mounting
ever replaces this one, that calibration is no longer valid and needs
redoing: run a real checkerboard calibration (ros2 run camera_calibration
cameracalibrator) with the camera mounted on the arm AT THE RESOLUTION
THAT WILL ACTUALLY BE USED IN PRODUCTION, then either overwrite that
file or pass a different yaml's file:// URI via the camera_info_url
launch argument. Pass camera_info_url:="" to go back to uncalibrated
(all-zero distortion/intrinsics) for quick bringup/eyeballing the image
-- NOT fine for anything that trusts the camera's pose output: ArUco/
panel pose estimation needs real intrinsics, and uncalibrated numbers
will silently produce wrong poses, not an obvious error.

USB backend has a `disable_autofocus` argument to disable continuous
autofocus and pin a fixed `focus_absolute` instead (confirmed live:
this camera's `focus_automatic_continuous` control hunts/refocuses
mid-stream, which produces intermittent blur, not a resolution/bitrate
problem) -- but it currently defaults to *false* (autofocus left ON),
by choice, not because the fix doesn't work: focus_absolute=450 below
is a confirmed-good fixed value from earlier testing, kept here ready
to use, just not applied by default right now. Flip disable_autofocus
to true to use it again. Exposure itself is always left on auto
(confirmed live: manual exposure wasn't needed once focus was
addressed); `brightness` is set instead as a plain offset on top of
whatever auto exposure lands on. Both focus_absolute and brightness
depend on the camera's actual working distance/lighting once mounted on
the arm; there's no good way to determine either from a dev-machine test
on a different setup, so tune them live once mounted: `ros2 run
rqt_image_view rqt_image_view /camera/image_raw` alongside `v4l2-ctl -d
<video_device> -c focus_absolute=<0-1023>` / `-c brightness=<0-255>`
until it looks right, then set those values here as the defaults. Both
are CSI-backend N/A -- nvarguscamerasrc/Jetson CSI modules control
focus/exposure through an entirely different, driver-specific mechanism,
not V4L2 controls.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    backend = LaunchConfiguration("backend")
    video_device = LaunchConfiguration("video_device")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    csi_sensor_id = LaunchConfiguration("csi_sensor_id")
    csi_width = LaunchConfiguration("csi_width")
    csi_height = LaunchConfiguration("csi_height")
    frame_id = LaunchConfiguration("frame_id")
    camera_info_url = LaunchConfiguration("camera_info_url")
    disable_autofocus = LaunchConfiguration("disable_autofocus")
    focus_absolute = LaunchConfiguration("focus_absolute")
    brightness = LaunchConfiguration("brightness")
    backlight_compensation = LaunchConfiguration("backlight_compensation")

    is_usb = PythonExpression(["'", backend, "' == 'usb'"])
    is_csi = PythonExpression(["'", backend, "' == 'csi'"])
    should_fix_focus = PythonExpression([is_usb, " and '", disable_autofocus, "' == 'true'"])

    # V4L2 controls are independent of whatever has the device open for
    # streaming -- confirmed live, this applies fine whether gscam has
    # already started or not, so no ordering dependency on camera_usb.
    # Must be two SEPARATE v4l2-ctl invocations, not one call with both
    # -c flags: confirmed live that setting focus_automatic_continuous=0
    # and focus_absolute=N in the same ioctl batch fails ("Permission
    # denied") on this camera -- focus_absolute is only a writable
    # control once autofocus has actually been turned off first. Two
    # ExecuteProcess actions chained via OnProcessExit (rather than one
    # ExecuteProcess running "bash -c 'cmd1 && cmd2'") makes that
    # required order an explicit launch-graph dependency instead of
    # shell interpolation -- the same pattern arm_gazebo.launch.py
    # already uses for delayed_controller_spawners/delayed_move_group.
    disable_autofocus_process = ExecuteProcess(
        cmd=["v4l2-ctl", "-d", video_device, "-c", "focus_automatic_continuous=0"],
        condition=IfCondition(should_fix_focus),
    )
    set_focus_absolute_process = ExecuteProcess(
        cmd=["v4l2-ctl", "-d", video_device, "-c", ["focus_absolute=", focus_absolute]],
        condition=IfCondition(should_fix_focus),
    )

    def _set_focus_absolute_if_disable_succeeded(event, context):
        # Mirrors the original "cmd1 && cmd2" semantics exactly: only
        # attempt focus_absolute if focus_automatic_continuous=0 actually
        # succeeded. A plain on_exit=[...] list fires on ANY exit code,
        # success or failure -- attempting focus_absolute after a failed
        # disable is pointless (it's only a writable control once
        # autofocus is actually off, see comment above) and would just
        # log a second, confusing failure for the same root cause.
        if event.returncode == 0:
            return [set_focus_absolute_process]
        return None

    fix_focus = RegisterEventHandler(
        OnProcessExit(
            target_action=disable_autofocus_process,
            on_exit=_set_focus_absolute_if_disable_succeeded,
        )
    )
    # brightness/backlight_compensation have no auto-mode gate (always
    # writable, unlike focus_absolute), so one v4l2-ctl call for both is
    # fine. backlight_compensation=0 confirmed live: default (1) boosts
    # exposure on high-contrast scenes, which pushed bright light sources
    # further into blown-out white instead of helping.
    fix_brightness = ExecuteProcess(
        cmd=[
            "v4l2-ctl", "-d", video_device,
            "-c", ["brightness=", brightness, ",backlight_compensation=", backlight_compensation],
        ],
        condition=IfCondition(is_usb),
    )

    usb_gscam_config = [
        "v4l2src device=", video_device,
        " ! video/x-raw,format=YUY2,width=", width, ",height=", height,
        " ! videoconvert",
    ]
    # format=NV12 explicit rather than left to negotiate -- that's what
    # nvarguscamerasrc's NVMM buffers actually are on Jetson, and leaving
    # it implicit is exactly the kind of thing that's easy to get away
    # with on one Jetson/L4T version and silently fail to negotiate on
    # another. Still unverified on real hardware either way -- see
    # module docstring.
    csi_gscam_config = [
        "nvarguscamerasrc sensor-id=", csi_sensor_id,
        " ! video/x-raw(memory:NVMM),format=NV12,width=", csi_width, ",height=", csi_height, ",framerate=30/1",
        " ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert",
    ]

    camera_usb = Node(
        package="gscam",
        executable="gscam_node",
        name="arm_camera",
        output="screen",
        parameters=[{
            "gscam_config": usb_gscam_config,
            "frame_id": frame_id,
            "camera_info_url": camera_info_url,
            "use_gst_timestamps": False,
        }],
        condition=IfCondition(is_usb),
    )

    camera_csi = Node(
        package="gscam",
        executable="gscam_node",
        name="arm_camera",
        output="screen",
        parameters=[{
            "gscam_config": csi_gscam_config,
            "frame_id": frame_id,
            "camera_info_url": camera_info_url,
            "use_gst_timestamps": False,
        }],
        condition=IfCondition(is_csi),
    )

    unknown_backend_guard = Node(
        package="gscam",
        executable="gscam_node",
        name="arm_camera",
        parameters=[{"gscam_config": "INVALID_BACKEND_MUST_BE_usb_OR_csi"}],
        condition=UnlessCondition(PythonExpression([is_usb, " or ", is_csi])),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "backend", default_value="usb",
            description="'usb' (v4l2src, dev-machine testing -- the only backend confirmed working) "
                        "or 'csi' (nvarguscamerasrc, real Jetson -- EXPERIMENTAL, never run on real "
                        "CSI hardware, see module docstring before trusting it on the real arm)."),
        DeclareLaunchArgument(
            "video_device",
            default_value="/dev/v4l/by-id/usb-HZ_USB_Camera_20220301104-video-index0",
            description="Stable by-id path (udev's own 60-persistent-v4l.rules, keyed on this "
                        "camera's USB serial) rather than /dev/videoN, whose number isn't stable "
                        "across reboots/replugs/other USB devices. The repo's example dev container "
                        "config (docker/docker-compose.dev.example.yaml) already mounts /dev:/dev, "
                        "which covers this path with no extra configuration. Only matches this "
                        "specific physical camera unit -- override if swapped for a different one."),
        DeclareLaunchArgument(
            "width", default_value="640",
            description="USB backend uses raw YUYV, not MJPEG -- see module docstring for why. "
                        "640x480 gets this camera's full ~30fps in YUYV; 1280x720 only manages "
                        "~5-6fps in practice (USB2.0 bandwidth). Only raise this with that ceiling in mind."),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("csi_sensor_id", default_value="0"),
        DeclareLaunchArgument(
            "csi_width", default_value="1280",
            description="EXPERIMENTAL/unverified backend, see module docstring. Defaults to a "
                        "practical CV-pipeline resolution rather than the OV5693's native 2592 -- "
                        "raise it (together with csi_height) if the full 5MP is actually needed; "
                        "exact supported resolution/framerate combos still need verifying on real "
                        "Jetson hardware."),
        DeclareLaunchArgument("csi_height", default_value="720"),
        DeclareLaunchArgument("frame_id", default_value="arm_camera_optical_frame"),
        DeclareLaunchArgument(
            "camera_info_url",
            # Resolved via the ROS package index at launch time (this
            # package's own installed config/, not a path into the repo
            # checkout) -- works regardless of where/how the workspace is
            # mounted, unlike a literal filesystem path baked in here.
            default_value=[
                "file://",
                PathJoinSubstitution([FindPackageShare("arm_sensors"), "config", "ov5693_usb.yaml"]),
            ],
            description="Set to \"\" for uncalibrated (all-zero intrinsics) bringup"),
        DeclareLaunchArgument(
            "disable_autofocus", default_value="false",
            description="USB backend only: disable continuous autofocus hunting and pin focus_absolute "
                        "instead. Left on (autofocus enabled) for now by choice, even though "
                        "focus_absolute=450 below is a confirmed-good fixed value -- flip this back "
                        "to true to use it again, no need to re-tune."),
        DeclareLaunchArgument(
            "focus_absolute", default_value="450",
            description="0-1023, lens position. Only applied when disable_autofocus=true (currently "
                        "isn't, see above). 450 confirmed sharp for the current dev-machine test "
                        "distance -- re-tune if the camera's real mounted working distance on the "
                        "arm ends up meaningfully different, see module docstring."),
        DeclareLaunchArgument(
            "brightness", default_value="110",
            description="0-255. 110 confirmed good for current indoor test lighting (auto exposure "
                        "left on) -- re-tune if the real lighting on the arm differs, see module "
                        "docstring."),
        DeclareLaunchArgument(
            "backlight_compensation", default_value="0",
            description="0-2. Default (1) was confirmed live to boost exposure on high-contrast "
                        "scenes, pushing bright light sources further into blown-out white instead "
                        "of helping -- 0 disables that."),
        disable_autofocus_process,
        fix_focus,
        fix_brightness,
        camera_usb,
        camera_csi,
        unknown_backend_guard,
    ])
