"""Updates the emulator's firmware over its USB serial port - no BOOT/EN buttons needed.

    python tools/serial_update.py COM8 build/ELM327_Emulator.ino.bin

The emulator must already run a firmware with the ATPLUPD command and the "min_spiffs"
partition scheme (two app partitions). Protocol: "ATPLUPD<size>,<md5>" at 38400 baud, the
emulator answers "READY" and switches to 460800 baud, then the image follows in blocks of
1024 bytes, each acknowledged with "K". At the end the emulator checks the MD5, answers "DONE"
and restarts into the new firmware; on any error it keeps the old one.
Needs pyserial.
"""
import hashlib
import sys
import time

import serial

ELM_BAUD, UPDATE_BAUD, BLOCK = 38400, 460800, 1024


def read_until(port, words, timeout):
    """Reads until one of the words appears; returns the text read."""
    text, end = b"", time.time() + timeout
    while time.time() < end:
        text += port.read(port.in_waiting or 1)
        for w in words:
            if w.encode() in text:
                return text.decode(errors="replace")
    return text.decode(errors="replace")


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    image = open(sys.argv[2], "rb").read()
    md5 = hashlib.md5(image).hexdigest()

    port = serial.Serial()
    port.port, port.baudrate, port.timeout = sys.argv[1], ELM_BAUD, 0.1
    port.dtr = port.rts = False   # RTS is wired to EN on these boards: do not reset the emulator
    port.open()
    port.write(b"\rATE0\r")
    read_until(port, [">"], 1)
    port.reset_input_buffer()
    port.write(f"ATPLUPD{len(image)},{md5}\r".encode())
    answer = read_until(port, ["READY", "ERR", "?"], 5)
    if "READY" not in answer:
        sys.exit("Emulator refused the update: " + answer.strip())

    time.sleep(0.1)
    port.baudrate = UPDATE_BAUD
    start = time.time()
    for at in range(0, len(image), BLOCK):
        port.write(image[at:at + BLOCK])
        ack = read_until(port, ["K", "ERR"], 5)
        if "K" not in ack:
            sys.exit(f"\nNo acknowledgement at {at} bytes: {ack.strip()}")
        done = min(at + BLOCK, len(image))
        print(f"\r{done * 100 // len(image):3d} %  {done} of {len(image)} bytes", end="", flush=True)
    result = read_until(port, ["DONE", "ERR"], 30)
    port.close()
    print()
    if "DONE" not in result:
        sys.exit("Update failed: " + result.strip())
    print(f"Updated in {time.time() - start:.0f} s, MD5 {md5} verified - the emulator restarts.")


if __name__ == "__main__":
    main()
