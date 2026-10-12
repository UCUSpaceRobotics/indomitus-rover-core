# What the arm's motors are capable of right now, as arm_power last confirmed
# it with controller_manager — the arm's counterpart of DriveState.
#
# Published latched on /arm/state. Every arm operator (keyboard, gamepad,
# ground station) reads it instead of asking controller_manager itself.

# Hardware component ArmSystem is ACTIVE (motors enabled). False: the real
# arm's motors are read but give no torque.
bool motors_enabled

# The arm's trajectory or streaming controller is active, i.e. the arm
# follows commands.
bool controller_active

# The stiffness ramp after activation is over, so commanded motion is followed
# without lagging.
bool stiffness_ready

# The measured pose is the park pose (resting on its support), so the motors
# may be turned off.
bool in_park_pose

# arm_power is switching the motors on or off right now.
bool busy

# motors_enabled and controller_active and stiffness_ready.
bool can_move

# Why the last power request failed, e.g. a release aborted because the arm
# moved while relaxing. Empty after a success.
string last_error
