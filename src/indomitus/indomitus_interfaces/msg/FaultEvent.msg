# A fault transition detected by whichever component owns the hardware.
#
# Published on /fault_events by the detector itself (e.g. the chassis
# hardware_interface), never derived downstream from sampled state — only the
# owning component sees every frame and can report the exact moment and code.
#
# Events are edge-triggered: one message per transition, never periodic state.
# For "what is the health right now", use /diagnostics instead.

std_msgs/Header header

# ── What happened ─────────────────────────────────────────────────────────────
uint8 EVENT_FAULT_ENTER   = 0   # healthy -> faulted
uint8 EVENT_FAULT_CHANGE  = 1   # faulted -> different fault
uint8 EVENT_FAULT_CLEAR   = 2   # faulted -> healthy
uint8 EVENT_SIGNAL_LOST   = 3   # feedback stopped arriving
uint8 EVENT_SIGNAL_OK     = 4   # feedback resumed
uint8 event

# ── Who it happened to ────────────────────────────────────────────────────────
string component      # e.g. "chassis/drive_FL" — stable across vendors
string joint_name     # URDF joint, empty if not a joint
string vendor         # "damiao" | "steadywin" | ...
uint8  esc_id         # CAN device address

# ── Normalised fault (see rover_fault::Fault) ─────────────────────────────────
uint8 FAULT_NONE         = 0
uint8 FAULT_NOT_ENABLED  = 1
uint8 FAULT_COMM_LOSS    = 2
uint8 FAULT_UNDERVOLTAGE = 3
uint8 FAULT_OVERVOLTAGE  = 4
uint8 FAULT_VOLTAGE      = 5    # over/under indistinguishable
uint8 FAULT_OVERCURRENT  = 6
uint8 FAULT_OVERTEMP     = 7
uint8 FAULT_OVERLOAD     = 8
uint8 FAULT_ENCODER      = 9
uint8 FAULT_HARDWARE     = 10
uint8 FAULT_SOFTWARE     = 11
uint8 FAULT_UNKNOWN      = 12
# Transport-level: reported against the CAN interface, not against a motor.
uint8 FAULT_CAN_ERROR_PASSIVE = 13
uint8 FAULT_CAN_BUS_OFF       = 14
uint8 fault
uint8 previous_fault
string fault_name     # human-readable, so logs stay readable without the enum

# ── Recovery policy (see rover_fault::Recovery) ───────────────────────────────
uint8 RECOVERY_NONE      = 0
uint8 RECOVERY_IMMEDIATE = 1
uint8 RECOVERY_LIMITED   = 2
uint8 RECOVERY_BACKOFF   = 3
uint8 RECOVERY_THERMAL   = 4
uint8 RECOVERY_LATCH     = 5
uint8 recovery

# ── Forensics ─────────────────────────────────────────────────────────────────
uint8 raw_code        # untranslated vendor code, for post-hoc analysis

# Freeze frame: motor state at the instant of the transition. This is the whole
# point of the event — the values that explain *why*, captured before they
# recover. NaN where the vendor does not report the quantity.
float32 position
float32 velocity
float32 torque
float32 temperature
float32 voltage
float32 current
uint8   mode
bool    enabled
