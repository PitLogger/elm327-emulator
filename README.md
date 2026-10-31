# ELM327 emulator for ESP32

An ESP32 that behaves like an ELM327 OBD-II adapter connected to a vehicle with a CAN bus
(ISO 15765-4, 11 bit, 500 kbit/s). Develop and test OBD apps, diagnostic tools and your own
projects at your desk – no car needed.

Created as part of the PitLogger project, but stands on its own and works with any
ELM327-compatible software.

## Features

- **Three interfaces at the same time**
  - USB serial: 38400 baud, 8N1, line ending CR
  - Bluetooth Classic (SPP) as `OBDII`, PIN `1234` – like cheap ELM327 adapters
  - Bluetooth Low Energy as `OBDII`: service `FFF0`, responses via notify on `FFF1`,
    commands via write on `FFF2` – like Vgate / Veepeak BLE adapters
- **Simulated drive profile:** speed swings between 0 and 120 km/h every 90 s, engine speed,
  load, mass air flow etc. follow, the engine slowly warms up, the tank empties
- **AT commands:** `Z`, `WS`, `D`, `I`, `@1`, `E0/1`, `L0/1`, `S0/1`, `H0/1`, `SP`/`TP`, `DP`, `DPN`,
  `RV`, `MA` (monitor all); others are acknowledged with `OK`
- **Mode 01** live data, several PIDs per request (e.g. `010C0D11`):
  `01 04 05 06 07 0B 0C 0D 0E 0F 10 11 1C 1F 21 2F 33 42 46 51 5C`
- **Mode 03 / 07 / 0A** trouble codes (stored: `P0133`, `P0301`), **mode 04** clear codes
- **Mode 09** VIN (multi-frame / ISO-TP) and ECU name
- Responses with/without headers (`ATH1`/`ATH0`) and with/without spaces (`ATS1`/`ATS0`)

## Hardware

- ESP32 **with Bluetooth Classic**, i.e. the original ESP32 (e.g. ESP32-DevKitC,
  ESP32-WROOM-32). ESP32-S2/S3/C3 have no Bluetooth Classic.
- Optional: **GPIO 13 to GND** makes the emulator invisible (drops connections, no Bluetooth
  discoverability). Left open = Bluetooth on. The onboard LED on GPIO 2 shows the state.

## Build and flash

Requires [arduino-cli](https://arduino.github.io/arduino-cli/) or the Arduino IDE with
Espressif's ESP32 core (tested with **esp32:esp32 3.3.12**). No other libraries needed.

```bash
arduino-cli compile --upload -p COM3 ELM327_Emulator
```

Board and partition scheme (`huge_app`, Classic + BLE do not fit the default scheme) are set
in `ELM327_Emulator/sketch.yaml`. In the Arduino IDE: board "ESP32 Dev Module",
partition scheme "Huge APP (3MB No OTA/1MB SPIFFS)".

**Board does not enter download mode** (`Wrong boot mode detected`)? Some boards have no
working auto-reset circuit. Hold BOOT, press EN briefly, release BOOT and flash without
reset, e.g.:

```bash
esptool --chip esp32 --port COM3 --before no-reset write-flash 0x10000 ELM327_Emulator.ino.bin
```

## Testing

`tools/elm_test.py` sends a series of typical commands over USB and prints the responses:

```bash
pip install pyserial
python tools/elm_test.py COM3
```

Over Bluetooth, any OBD app works (e.g. Car Scanner, Torque) – pick the adapter "OBDII".

## License

MIT, see [LICENSE](LICENSE).
