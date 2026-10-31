"""Small test client for the ESP32 ELM327 emulator.

Usage:    python tools/elm_test.py [PORT] [BAUD]
Defaults: COM3, 38400 baud
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 38400

COMMANDS = [
    "ATZ", "ATE0", "ATL0", "ATS1", "ATH0", "ATSP0", "ATI", "ATDP", "ATDPN", "ATRV",
    "0100", "0120", "0140", "010C", "010D", "0105", "010C0D11",
    "0101", "03", "0902", "090A", "ATH1", "0902", "010C", "ATH0", "04", "03", "22F190",
]


def send(ser: serial.Serial, cmd: str, timeout: float = 3.0) -> str:
    ser.write((cmd + "\r").encode("ascii"))
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        buf += ser.read(ser.in_waiting or 1)
        if buf.endswith(b">"):
            break
    return buf.decode("ascii", errors="replace")


def main() -> None:
    # dsrdtr/rtscts off, so that opening the port ideally does not reset the ESP32
    with serial.Serial(PORT, BAUD, timeout=0.1, dsrdtr=False, rtscts=False) as ser:
        time.sleep(1.5)  # in case the ESP32 restarts when the port is opened
        ser.reset_input_buffer()
        for cmd in COMMANDS:
            reply = send(ser, cmd)
            lines = [l for l in reply.replace("\n", "").split("\r") if l and l != ">"]
            print(f"{cmd:>10} -> " + " | ".join(lines))


if __name__ == "__main__":
    main()
