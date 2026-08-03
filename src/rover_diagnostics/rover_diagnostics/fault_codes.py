"""Vendor fault-code decoding for the rover's motors.

Each vendor encodes faults differently, so this module is the one place that
knows about those encodings. Everything downstream works with the normalized
names produced here.

Damiao encodes state and fault in a single 4-bit nibble, where 0x0 (disabled)
and 0x1 (enabled) are *normal states*, not faults. Steadywin uses an
independent fault bitmask plus a separate operating-mode byte, where mode 0
("off state") is how it reports that it has stopped accepting commands.
"""

# Damiao ERR nibble (manual p.9). 0x0/0x1 are states, 0x8-0xE are faults.
DAMIAO_ENABLED = 0x1
DAMIAO_DISABLED = 0x0

_DAMIAO_FAULTS = {
    0x8: 'overvoltage',
    0x9: 'undervoltage',
    0xA: 'overcurrent',
    0xB: 'mos_overtemperature',
    0xC: 'coil_overtemperature',
    0xD: 'communication_loss',
    0xE: 'overload',
}

# Steadywin fault bitmask (CAN protocol V3.06b0, command 0xAE byte[7]).
_STEADYWIN_BITS = [
    (0x01, 'voltage'),
    (0x02, 'current'),
    (0x04, 'temperature'),
    (0x08, 'encoder'),
    (0x40, 'hardware'),
    (0x80, 'software'),
]

# Steadywin operating mode (0xAE byte[6]).
STEADYWIN_MODE_OFF = 0


def decode_damiao(fault_code, mode):
    """Return a list of fault names for a Damiao drive motor.

    ``fault_code`` carries the raw ERR nibble. Values 0x2-0x7 are reserved and
    should never appear; they are surfaced as unknown rather than ignored, so a
    firmware change cannot silently look healthy.
    """
    del mode  # Damiao reports state and fault in the same nibble.
    if fault_code in (DAMIAO_DISABLED, DAMIAO_ENABLED):
        return []
    if fault_code in _DAMIAO_FAULTS:
        return [_DAMIAO_FAULTS[fault_code]]
    return ['unknown_0x{:02X}'.format(fault_code)]


def decode_steadywin(fault_code, mode):
    """Return a list of fault names for a Steadywin steer motor.

    Several bits can be set at once, so all of them are reported. A motor that
    has dropped to mode 0 with no fault bits set has silently stopped accepting
    commands, which is reported as ``not_enabled``.
    """
    faults = [name for bit, name in _STEADYWIN_BITS if fault_code & bit]
    if not faults and mode == STEADYWIN_MODE_OFF:
        return ['not_enabled']
    return faults


_DECODERS = {
    'damiao': decode_damiao,
    'steadywin': decode_steadywin,
}


def decode(motor_type, fault_code, mode):
    """Decode a raw vendor fault code into a list of normalized fault names.

    Returns an empty list when the motor is healthy. An unrecognized
    ``motor_type`` yields a single ``unknown_vendor`` entry rather than an
    empty list, so a typo in the driver cannot mask a real fault.
    """
    decoder = _DECODERS.get(motor_type)
    if decoder is None:
        return ['unknown_vendor:{}'.format(motor_type)]
    return decoder(int(fault_code), int(mode))
