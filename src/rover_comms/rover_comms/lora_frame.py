#!/usr/bin/env python3
"""Wire format for the mast <-> rover LoRa link.

    AA 55 | type | seq | payload[4] | crc16[2]        = 10 bytes

THIRD COPY. This format now lives in three places across two repositories,
with no shared source of truth:

  1. indomitus-ground-station  mast/lora_frame.py                     (mast Pi)
  2. indomitus-ground-station  microcontrollers_indomitus/
                               esp32s3_lora_rover/src/link_frame.h    (bench rig)
  3. here                                                             (rover)

Change one and the link breaks quietly: frames fail CRC, or worse, pass and
decode to the wrong values. test/test_lora_frame.py pins the vectors so this
copy cannot drift without CI noticing. The other two are checked by
`lora_bridge.py --selftest` and the firmware's TEST command, which print the
same ten bytes for the same input.

Duplicating rather than sharing a package is deliberate: the mast, the rover
and the bench rig are three machines that must not share a deployment, and a
ten-byte struct is cheaper to duplicate than to version across two repos.

Ten bytes fits inside the E32's 58-byte single-packet limit, so one frame is
always one air packet - write it in a single write() and the module will not
split it.

Stdlib only: this has to import cleanly with nothing but Python available.
"""

from typing import List, NamedTuple, Optional, Tuple

SYNC0 = 0xAA
SYNC1 = 0x55

PAYLOAD_LEN = 4
FRAME_LEN = 10

# Direction is fixed: TELEOP is mast->rover, STATUS is rover->mast.
TYPE_TELEOP = 0x01
TYPE_STATUS = 0x02

# Teleop flags
FLAG_ESTOP = 0x01
# Set by the mast, unread on the rover: this end drives what it is given and
# has no mode of its own. Defined because all three copies of the format must
# agree on the bit, not because anything here looks at it.
FLAG_MODE = 0x02

# Status flags
STATUS_FAILSAFE = 0x01
STATUS_ESTOP = 0x02


class Teleop(NamedTuple):
    """Velocities as percentages of the rover's maximum, -100..100."""

    vx: int = 0
    vy: int = 0
    wz: int = 0
    flags: int = 0


class Status(NamedTuple):
    """rx_ok / rx_bad are the low bytes of the rover's free-running counters."""

    echo_seq: int
    rx_ok: int
    rx_bad: int
    flags: int


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: init 0xFFFF, poly 0x1021, no reflection, no xorout."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _clamp_i8(value: int) -> int:
    return max(-128, min(127, int(value)))


def pack_teleop(cmd: Teleop) -> bytes:
    return bytes((
        _clamp_i8(cmd.vx) & 0xFF,
        _clamp_i8(cmd.vy) & 0xFF,
        _clamp_i8(cmd.wz) & 0xFF,
        cmd.flags & 0xFF,
    ))


def unpack_teleop(payload: bytes) -> Teleop:
    def signed(value: int) -> int:
        return value - 256 if value > 127 else value

    return Teleop(signed(payload[0]), signed(payload[1]), signed(payload[2]), payload[3])


def pack_status(status: Status) -> bytes:
    return bytes((status.echo_seq & 0xFF, status.rx_ok & 0xFF,
                  status.rx_bad & 0xFF, status.flags & 0xFF))


def unpack_status(payload: bytes) -> Status:
    return Status(payload[0], payload[1], payload[2], payload[3])


def encode(frame_type: int, seq: int, payload: bytes) -> bytes:
    """Serialise one frame. `payload` must be exactly PAYLOAD_LEN bytes."""
    if len(payload) != PAYLOAD_LEN:
        raise ValueError(f"payload must be {PAYLOAD_LEN} bytes, got {len(payload)}")
    body = bytes((frame_type & 0xFF, seq & 0xFF)) + payload
    # The sync word is excluded: it is a framing marker, not data, and a
    # receiver that resynchronised mid-stream has not seen it.
    crc = crc16(body)
    return bytes((SYNC0, SYNC1)) + body + bytes((crc & 0xFF, crc >> 8))


class Parser:
    """Byte-at-a-time parser that resynchronises after corruption.

    Feed it everything the UART hands over; feed() returns the frames whose CRC
    checked out, as (type, seq, payload) tuples.
    """

    _WAIT_SYNC0, _WAIT_SYNC1, _BODY = 0, 1, 2

    def __init__(self) -> None:
        self.ok = 0
        self.bad = 0
        self._state = self._WAIT_SYNC0
        self._body = bytearray()

    def reset(self) -> None:
        """Drop half-received framing state, keeping the counters.

        Called when the serial port is reopened. Bytes captured before the
        port died must not be joined to bytes that arrive after it comes back:
        the join would almost certainly fail CRC, but "almost certainly" is
        not the guarantee wanted on the path that steers the rover. ok/bad are
        free-running totals reported in STATUS, so they survive the reset.
        """
        self._state = self._WAIT_SYNC0
        self._body.clear()

    def feed(self, chunk: bytes) -> List[Tuple[int, int, bytes]]:
        frames = []
        for byte in chunk:
            frame = self._push(byte)
            if frame is not None:
                frames.append(frame)
        return frames

    def _push(self, byte: int) -> Optional[Tuple[int, int, bytes]]:
        if self._state == self._WAIT_SYNC0:
            if byte == SYNC0:
                self._state = self._WAIT_SYNC1
            return None

        if self._state == self._WAIT_SYNC1:
            if byte == SYNC1:
                self._state = self._BODY
                self._body.clear()
            elif byte != SYNC0:
                # AA AA 55 is a legal start, so a repeated AA keeps us here
                # rather than throwing away the sync we already have.
                self._state = self._WAIT_SYNC0
            return None

        self._body.append(byte)
        if len(self._body) < FRAME_LEN - 2:
            return None
        self._state = self._WAIT_SYNC0

        body = bytes(self._body)
        want = body[6] | (body[7] << 8)
        if crc16(body[:2 + PAYLOAD_LEN]) != want:
            self.bad += 1
            return None
        self.ok += 1
        return body[0], body[1], body[2:2 + PAYLOAD_LEN]
