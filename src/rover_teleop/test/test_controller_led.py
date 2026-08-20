"""Light-bar colour rules and sysfs writing.

The failure these guard against is a light bar that lies: green while the rover
cannot actually be driven, or the driver's default blue left in place after a
reconnect, which an operator reads as 'yielding to navigation'.

No ROS import anywhere in here — controller_led is deliberately standalone.
"""

import os

import pytest

from rover_teleop.controller_led import (
    LED_CONTROLLER_OFF,
    LED_DRIVING,
    LED_MOTORS_INHIBITED,
    LED_MOTORS_OFF,
    LED_NAV,
    LED_NODE_DOWN,
    ControllerLed,
    led_colour,
)


class FakeLogger:
    def __init__(self):
        self.warnings = []
        self.debugs = []

    def warning(self, msg):
        self.warnings.append(msg)

    # rclpy loggers expose warn(); keep both so the fake matches either call.
    warn = warning

    def debug(self, msg):
        self.debugs.append(msg)


def make_dualsense(root, name='input19:rgb:indicator'):
    """A stand-in for the sysfs tree hid-playstation exposes for a DualSense."""
    base = root / name
    base.mkdir(parents=True)
    (base / 'multi_intensity').write_text('0 0 255\n')
    (base / 'brightness').write_text('255\n')
    (base / 'max_brightness').write_text('255\n')
    return base


def make_dualshock(root, prefix='input19'):
    """The other shape the same driver exposes: separate channels + a gate."""
    made = {}
    for name, max_brightness in (('red', '255'), ('green', '255'),
                                 ('blue', '255'), ('global', '1')):
        base = root / f'{prefix}:{name}'
        base.mkdir(parents=True)
        (base / 'brightness').write_text('0\n')
        (base / 'max_brightness').write_text(f'{max_brightness}\n')
        made[name] = base
    return made


def led_for(root, **kwargs):
    return ControllerLed(
        FakeLogger(),
        dualsense_glob=str(root / '*:rgb:indicator'),
        dualshock_glob=str(root / '*:global'),
        **kwargs,
    )


# ── colour priority ──────────────────────────────────────────────────────────

def test_motors_off_shows_red_whatever_else_is_true():
    # Most-serious state wins: nothing about the controller or the joystick
    # matters while the hardware is inactive.
    assert led_colour(motors_enabled=False, motors_inhibited=False,
                      controller_active=True, joystick_active=True) == LED_MOTORS_OFF
    assert led_colour(motors_enabled=False, motors_inhibited=True,
                      controller_active=False, joystick_active=False) == LED_MOTORS_OFF


def test_controller_inactive_shows_orange():
    assert led_colour(motors_enabled=True, motors_inhibited=False,
                      controller_active=False, joystick_active=True) == LED_CONTROLLER_OFF


def test_yielding_to_nav_shows_blue():
    assert led_colour(motors_enabled=True, motors_inhibited=False,
                      controller_active=True, joystick_active=False) == LED_NAV


def test_joystick_in_command_shows_green():
    assert led_colour(motors_enabled=True, motors_inhibited=False,
                      controller_active=True, joystick_active=True) == LED_DRIVING


def test_inhibited_outranks_everything_below_motors_off():
    # After clear_motor_errors the interpreter still believes the motors are
    # enabled and the controller active, but the hardware will not drive until
    # the motor button is cycled. Green here would promise control that the
    # operator does not have.
    assert led_colour(motors_enabled=True, motors_inhibited=True,
                      controller_active=True, joystick_active=True) == LED_MOTORS_INHIBITED
    assert led_colour(motors_enabled=True, motors_inhibited=True,
                      controller_active=False, joystick_active=False) == LED_MOTORS_INHIBITED


def test_every_colour_is_distinguishable():
    # Two states sharing a colour would be indistinguishable on the bar, and
    # the mistake is invisible in review — assert it instead.
    colours = [LED_MOTORS_OFF, LED_MOTORS_INHIBITED, LED_CONTROLLER_OFF,
               LED_NAV, LED_DRIVING, LED_NODE_DOWN]
    assert len(set(colours)) == len(colours)


def test_node_down_is_not_a_drive_state():
    # LED_NODE_DOWN is painted on shutdown only; led_colour must never pick it,
    # or a live rover would claim its interpreter is gone.
    reachable = {
        led_colour(motors_enabled=m, motors_inhibited=i,
                   controller_active=c, joystick_active=a)
        for m in (True, False) for i in (True, False)
        for c in (True, False) for a in (True, False)
    }
    assert LED_NODE_DOWN not in reachable


# ── writing: DualSense ───────────────────────────────────────────────────────

def test_dualsense_write_sets_intensity_and_brightness(tmp_path):
    base = make_dualsense(tmp_path)
    led = led_for(tmp_path)

    assert led.set(LED_DRIVING) is True
    assert (base / 'multi_intensity').read_text() == '0 255 0'
    # Output is intensity × brightness; leaving brightness where a reconnect
    # put it can leave the bar dark no matter what the intensities say.
    assert (base / 'brightness').read_text() == '255'


def test_dualsense_brightness_follows_the_device_maximum(tmp_path):
    base = make_dualsense(tmp_path)
    (base / 'max_brightness').write_text('63\n')

    led_for(tmp_path).set(LED_NAV)
    assert (base / 'brightness').read_text() == '63'


def test_reconnect_under_a_new_input_index_is_still_found(tmp_path):
    # The index changes on every reconnect, which is why the path is globbed
    # per repaint instead of resolved once at startup.
    make_dualsense(tmp_path, name='input19:rgb:indicator')
    led = led_for(tmp_path)
    led.set(LED_DRIVING)

    for child in tmp_path.iterdir():
        for f in child.iterdir():
            f.unlink()
        child.rmdir()
    later = make_dualsense(tmp_path, name='input27:rgb:indicator')

    assert led.set(LED_NAV) is True
    assert (later / 'multi_intensity').read_text() == '0 0 255'


# ── writing: DualShock 4 ─────────────────────────────────────────────────────

def test_dualshock_write_sets_each_channel_and_the_gate(tmp_path):
    leds = make_dualshock(tmp_path)

    assert led_for(tmp_path).set(LED_CONTROLLER_OFF) is True
    assert (leds['red'] / 'brightness').read_text() == '255'
    assert (leds['green'] / 'brightness').read_text() == '120'
    assert (leds['blue'] / 'brightness').read_text() == '0'
    # ':global' gates the whole bar and maxes out at 1, not 255.
    assert (leds['global'] / 'brightness').read_text() == '1'


def test_global_without_colour_channels_is_not_treated_as_a_light_bar(tmp_path):
    # ':global' is not a PlayStation-only name; an unrelated LED must not be
    # written to just because it matched the glob.
    lonely = tmp_path / 'somethingelse:global'
    lonely.mkdir()
    (lonely / 'brightness').write_text('7\n')
    (lonely / 'max_brightness').write_text('255\n')

    assert led_for(tmp_path).set(LED_DRIVING) is False
    assert (lonely / 'brightness').read_text() == '7\n'


# ── failure isolation ────────────────────────────────────────────────────────

def test_no_light_bar_is_silent(tmp_path):
    # An Xbox pad, or any controller without an RGB bar. Normal setup, not a
    # misconfiguration — it must not produce warnings.
    led = led_for(tmp_path)
    assert led.set(LED_DRIVING) is False
    assert led._log.warnings == []


@pytest.mark.skipif(os.geteuid() == 0, reason='root ignores file permissions')
def test_unwritable_light_bar_warns_once_not_every_attempt(tmp_path):
    # The udev rule is missing or the user is not in plugdev. Logging this only
    # at debug level makes a real misconfiguration invisible in the field.
    base = make_dualsense(tmp_path)
    (base / 'multi_intensity').chmod(0o444)
    (base / 'brightness').chmod(0o444)

    led = led_for(tmp_path)
    assert led.set(LED_DRIVING) is False
    # A repaint timer means this runs forever; one warning per failure kind is
    # the difference between a usable log and an unreadable one.
    for _ in range(50):
        assert led.repaint() is False

    assert len(led._log.warnings) == 1
    assert 'plugdev' in led._log.warnings[0]
    assert len(led._log.debugs) == 1


@pytest.mark.skipif(os.geteuid() == 0, reason='root ignores file permissions')
def test_write_failure_never_escapes(tmp_path):
    # Teleop must survive a broken light bar. Nothing in here may raise.
    base = make_dualsense(tmp_path)
    (base / 'multi_intensity').chmod(0o444)
    (base / 'brightness').chmod(0o444)
    led = led_for(tmp_path)

    assert led.set(LED_DRIVING) is False
    assert led.repaint() is False


# ── periodic repaint ─────────────────────────────────────────────────────────

def test_repaint_restores_a_colour_someone_else_overwrote(tmp_path):
    # The reason the repaint is unconditional. The kernel driver and SDL (in
    # game_controller_node) both paint the bar when a controller enumerates,
    # and SDL's choice for player 1 is blue — indistinguishable from this
    # node's 'yielding to navigation'. Nothing tells us it happened, so the
    # only defence is to keep rewriting.
    base = make_dualsense(tmp_path)
    led = led_for(tmp_path)
    led.set(LED_DRIVING)

    (base / 'multi_intensity').write_text('0 0 255')

    assert led.repaint() is True
    assert (base / 'multi_intensity').read_text() == '0 255 0'


@pytest.mark.skipif(os.geteuid() == 0, reason='root ignores file permissions')
def test_repaint_recovers_once_udev_grants_write_access(tmp_path):
    # A controller that has just been plugged in races us: the first Joy
    # message can arrive before udev has applied the group write bit.
    base = make_dualsense(tmp_path)
    (base / 'multi_intensity').chmod(0o444)
    (base / 'brightness').chmod(0o444)

    led = led_for(tmp_path)
    assert led.set(LED_DRIVING) is False

    # udev catches up.
    (base / 'multi_intensity').chmod(0o644)
    (base / 'brightness').chmod(0o644)

    assert led.repaint() is True
    assert (base / 'multi_intensity').read_text() == '0 255 0'


def test_repaint_picks_up_a_device_that_appears_late(tmp_path):
    # Same race, earlier: the LED device itself has not been created yet.
    led = led_for(tmp_path)
    assert led.set(LED_DRIVING) is False

    base = make_dualsense(tmp_path)
    assert led.repaint() is True
    assert (base / 'multi_intensity').read_text() == '0 255 0'


def test_repaint_before_any_colour_is_set_does_nothing(tmp_path):
    make_dualsense(tmp_path)
    assert led_for(tmp_path).repaint() is False


def test_repaint_keeps_working_with_no_light_bar(tmp_path):
    # An Xbox pad: repaint runs forever at 2 Hz and must stay silent and cheap.
    led = led_for(tmp_path)
    led.set(LED_DRIVING)

    for _ in range(20):
        assert led.repaint() is False

    assert led._log.warnings == []
