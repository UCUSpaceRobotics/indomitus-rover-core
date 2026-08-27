#!/usr/bin/env python3
"""Prove which physical header pins a /dev/ttyTHS* actually is.

Jumper the header's TXD and RXD pins together, unplug nothing else, and run
this. Bytes written come straight back if - and only if - the device you named
is the UART on those two pins.

Worth doing before blaming a radio. A Jetson exposes several ttyTHS devices and
which one reaches the 40-pin header depends on the carrier board's device tree,
not on the numbering; a floating RX pin reads as a steady trickle of plausible
looking garbage, which is easy to mistake for a baud-rate problem.

    uart_loopback.py                    # /dev/ttyTHS1 at 9600
    uart_loopback.py /dev/ttyTHS2 115200

Reading it:
    echo      - this device is those two pins, wiring is fine
    silence   - wrong device, or the jumper is not making contact
    garbage   - RX is floating; nothing is driving it, jumper included
"""

import sys
import time

import serial

PROBE = b"\xAA\x55loopback\x0d\x0a"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyTHS1"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600

    with serial.Serial(port, baud, timeout=0) as ser:
        # Anything already queued is noise from before we started; a floating
        # pin will have been accumulating it for as long as the port existed.
        time.sleep(0.2)
        stale = ser.read(4096)
        ser.reset_input_buffer()

        ser.write(PROBE)
        ser.flush()

        got = b""
        deadline = time.time() + 2.0
        while len(got) < len(PROBE) and time.time() < deadline:
            got += ser.read(len(PROBE) - len(got))
            time.sleep(0.01)

    print(f"port      : {port} at {baud}")
    print(f"pre-noise : {len(stale)} bytes{'  <- RX is floating' if stale else ''}")
    print(f"sent      : {PROBE.hex(' ')}")
    print(f"received  : {got.hex(' ') if got else '(nothing)'}")

    if got == PROBE:
        print("\nLOOPBACK OK - this device is the pins you jumpered.")
        return 0
    if not got:
        print("\nNO ECHO - wrong device for those pins, or the jumper is not "
              "making contact. Try the other ttyTHS*.")
        return 1
    print("\nPARTIAL/CORRUPT - the pins are right but something is wrong with "
          "the line: check the baud and that nothing else drives it.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
