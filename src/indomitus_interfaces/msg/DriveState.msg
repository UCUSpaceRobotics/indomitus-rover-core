# What the rover's drive is actually capable of right now, as drive_power_node
# last confirmed it with controller_manager and the hardware interface.
#
# Published latched on drive/state. Both operators read it instead of keeping
# their own copy: the onboard joystick paints its light bar from it, the ground
# station UI paints its indicators from it. Neither can drift out of step with
# the hardware, and neither needs the other to be running.

# Hardware component RoverHardware is in the ACTIVE lifecycle state.
bool motors_enabled

# The swerve controller named below is active, i.e. /cmd_vel reaches the wheels.
bool controller_active

# Latched drive faults were cleared. The motors still report enabled, but the
# hardware will not drive until power is cycled off and on again.
bool motors_inhibited

# The swerve controller is in compact mode.
bool compact_mode

# True when every condition above is met, i.e. a twist on /cmd_vel moves the
# rover. Derived, but carried on the wire so no consumer has to re-derive it.
bool can_drive

# Whether joystick_interpreter is currently publishing to cmd_vel_joy. The
# joystick outranks the ground station in twist_mux, so this is the ground
# station operator's answer to "why is the rover ignoring my sticks?".
# False also when no joystick is running at all.
bool joystick_active

# Which swerve controller drive/power activates and deactivates.
string controller_name
