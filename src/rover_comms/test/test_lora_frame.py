"""Pin the LoRa wire format so this copy cannot drift from the other two.

The format lives in three files across two repositories (see lora_frame.py's
docstring). Nothing links them, so the only defence is that all three produce
identical bytes for identical input. These vectors are the same ones
`lora_bridge.py --selftest` prints on the mast and the ESP32 firmware's TEST
command prints on the bench rig.

If a test here fails, do not "fix" the expected bytes. The link is already
broken and the other two copies are what need changing.
"""

from rover_comms import lora_frame

# Captured from all three implementations, 2026-08-15.
TELEOP_SEQ0_10_M20_30 = bytes.fromhex("AA 55 01 00 0A EC 1E 00 37 D6".replace(" ", ""))
STATUS_SEQ7_ECHO6 = bytes.fromhex("AA 55 02 07 06 C8 01 00 7A 79".replace(" ", ""))


def test_teleop_encodes_to_the_agreed_bytes():
    frame = lora_frame.encode(
        lora_frame.TYPE_TELEOP, 0,
        lora_frame.pack_teleop(lora_frame.Teleop(10, -20, 30, 0)))
    assert frame == TELEOP_SEQ0_10_M20_30


def test_status_encodes_to_the_agreed_bytes():
    frame = lora_frame.encode(
        lora_frame.TYPE_STATUS, 7,
        lora_frame.pack_status(lora_frame.Status(6, 200, 1, 0)))
    assert frame == STATUS_SEQ7_ECHO6


def test_round_trip_recovers_signed_velocities():
    parser = lora_frame.Parser()
    got = parser.feed(TELEOP_SEQ0_10_M20_30)
    assert len(got) == 1
    frame_type, seq, payload = got[0]
    assert (frame_type, seq) == (lora_frame.TYPE_TELEOP, 0)
    assert lora_frame.unpack_teleop(payload) == lora_frame.Teleop(10, -20, 30, 0)


def test_corrupt_crc_is_rejected_not_delivered():
    corrupted = bytearray(TELEOP_SEQ0_10_M20_30)
    corrupted[-1] ^= 0xFF
    parser = lora_frame.Parser()
    assert parser.feed(bytes(corrupted)) == []
    assert (parser.ok, parser.bad) == (0, 1)


def test_resyncs_after_garbage():
    # A receiver that joined mid-stream, or lost bytes, must still find the
    # next frame rather than staying out of step forever.
    parser = lora_frame.Parser()
    got = parser.feed(b"\x00\xffrubbish\xaa\xaa" + TELEOP_SEQ0_10_M20_30)
    assert len(got) == 1
    assert parser.ok == 1


def test_frame_fits_one_air_packet():
    # The E32 sends up to 58 bytes as a single packet. Exceeding that would
    # split a frame across two transmissions and change the timing the whole
    # poll rate is built on.
    assert lora_frame.FRAME_LEN <= 58


def test_velocities_clamp_rather_than_wrap():
    # A percentage over 100 is a bug upstream, but it must not arrive as a
    # large negative number after the byte wraps.
    payload = lora_frame.pack_teleop(lora_frame.Teleop(200, -200, 0, 0))
    recovered = lora_frame.unpack_teleop(payload)
    assert recovered.vx == 127 and recovered.vy == -128


def test_reset_drops_a_partial_frame_but_keeps_the_counters():
    # The serial port can die mid-frame. What was captured before the reopen
    # must not be joined to what arrives after it.
    parser = lora_frame.Parser()
    parser.feed(TELEOP_SEQ0_10_M20_30)
    assert parser.ok == 1

    parser.feed(TELEOP_SEQ0_10_M20_30[:6])   # port dies part-way through
    parser.reset()

    got = parser.feed(TELEOP_SEQ0_10_M20_30)
    assert len(got) == 1, "the frame after the reconnect must parse cleanly"
    assert (parser.ok, parser.bad) == (2, 0), "counters are free-running"
