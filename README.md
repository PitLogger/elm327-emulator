# ELM327 emulator for ESP32

An ESP32 that behaves like an ELM327 OBD-II adapter connected to a vehicle with a CAN bus
(ISO 15765-4, 11 bit, 500 kbit/s) – and, with a CAN transceiver, like the vehicle itself on a
real CAN bus. Develop and test OBD apps, diagnostic tools, loggers and your own projects at your
desk – no car needed.

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
- **Real vehicles:** manufacturer requests (services 21/22) answered with answers recorded in
  real cars – **Hyundai i30 N 2021** (steering angle, wheel speeds, brake pressure, doors, …) and
  **Toyota bZ4X 2022** (traction battery, charging, …). Addressed with `ATSH` like on a real
  adapter, e.g. `ATSH7D1` then `220104`. `ATPLV` lists the vehicles, `ATPLV1` selects one
  (`0` = only the simulated VW Golf); the choice is kept across restarts
- **CAN bus:** with an SN65HVD230 transceiver the emulator answers on a real 500 kbit/s bus –
  OBD-II requests (`7DF` / `7E0` → `7E8`) and the recorded requests of the selected vehicle, with
  ISO-TP flow control. Ideal as the counterpart of a logger with a CAN interface

## Hardware

- ESP32 **with Bluetooth Classic**, i.e. the original ESP32 (e.g. ESP32-DevKitC,
  ESP32-WROOM-32). ESP32-S2/S3/C3 have no Bluetooth Classic.
- Optional: **CAN transceiver** SN65HVD230 (3.3 V): CTX ← GPIO 21, CRX → GPIO 22, 3V3, GND;
  CANH / CANL to the other node. Both ends of the bus need 120 Ω (the modules have it on board).
  Connect the grounds of both boards.
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

## Recorded vehicle data

`ELM327_Emulator/vehicles.h` is generated from the test cases of
[PitLogger/vehicle-profiles](https://github.com/PitLogger/vehicle-profiles), which were recorded by
[OBDb](https://github.com/OBDb) contributors:

```bash
python tools/gen_vehicles.py ../vehicle-profiles
```

## License

The code is MIT, see [LICENSE](LICENSE). The recorded vehicle answers in
`ELM327_Emulator/vehicles.h` come from OBDb and are licensed under
[CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).
