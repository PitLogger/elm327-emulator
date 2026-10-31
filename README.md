# ELM327-Emulator für ESP32

Ein ESP32 verhält sich wie ein ELM327-OBD-II-Adapter an einem Fahrzeug mit CAN-Bus
(ISO 15765-4, 11 bit, 500 kbit/s). Damit lassen sich OBD-Apps, Diagnose-Tools und eigene
Projekte am Schreibtisch entwickeln und testen – ganz ohne Auto.

Entstanden im PitLogger-Projekt, steht aber für sich allein und ist mit jeder
ELM327-kompatiblen Software nutzbar.

## Funktionen

- **Drei Schnittstellen gleichzeitig**
  - USB-Seriell: 38400 Baud, 8N1, Zeilenende CR
  - Bluetooth Classic (SPP) als `OBDII`, PIN `1234` – wie günstige ELM327-Adapter
  - Bluetooth Low Energy als `OBDII`: Service `FFF0`, Antworten per Notify auf `FFF1`,
    Befehle per Write auf `FFF2` – wie Vgate-/Veepeak-BLE-Adapter
- **Simuliertes Fahrprofil:** Geschwindigkeit pendelt alle 90 s zwischen 0 und 120 km/h,
  Drehzahl, Last, Luftmasse usw. laufen mit, der Motor wird langsam warm, der Tank leert sich
- **AT-Befehle:** `Z`, `WS`, `D`, `I`, `@1`, `E0/1`, `L0/1`, `S0/1`, `H0/1`, `SP`/`TP`, `DP`, `DPN`,
  `RV`, `MA` (Monitor All); weitere werden mit `OK` quittiert
- **Mode 01** Live-Daten, mehrere PIDs pro Anfrage (z.B. `010C0D11`):
  `01 04 05 06 07 0B 0C 0D 0E 0F 10 11 1C 1F 21 2F 33 42 46 51 5C`
- **Mode 03 / 07 / 0A** Fehlercodes (gespeichert: `P0133`, `P0301`), **Mode 04** Fehler löschen
- **Mode 09** Fahrgestellnummer (Multi-Frame / ISO-TP) und Steuergerätename
- Antworten mit/ohne Header (`ATH1`/`ATH0`) und mit/ohne Leerzeichen (`ATS1`/`ATS0`)

## Hardware

- ESP32 **mit Bluetooth Classic**, also das ursprüngliche ESP32 (z.B. ESP32-DevKitC, ESP32-WROOM-32).
  ESP32-S2/S3/C3 haben kein Bluetooth Classic.
- Optional: **GPIO 13 gegen GND** macht den Emulator unsichtbar (trennt Verbindungen, keine
  Bluetooth-Sichtbarkeit). Offen = Bluetooth an. Die Onboard-LED an GPIO 2 zeigt den Zustand.

## Bauen und Flashen

Benötigt [arduino-cli](https://arduino.github.io/arduino-cli/) oder die Arduino IDE mit dem
ESP32-Core von Espressif (getestet mit **esp32:esp32 3.3.12**). Weitere Bibliotheken sind nicht nötig.

```bash
arduino-cli compile --upload -p COM3 ELM327_Emulator
```

Board und Partitionsschema (`huge_app`, Classic + BLE passen nicht ins Standardschema) stehen in
`ELM327_Emulator/sketch.yaml`. In der Arduino IDE: Board „ESP32 Dev Module“,
Partition Scheme „Huge APP (3MB No OTA/1MB SPIFFS)“.

**Board geht nicht in den Download-Modus** (`Wrong boot mode detected`)? Manche Boards haben keine
funktionierende Auto-Reset-Schaltung. Dann BOOT gedrückt halten, kurz EN drücken, BOOT loslassen
und ohne Reset flashen, z.B.:

```bash
esptool --chip esp32 --port COM3 --before no-reset write-flash 0x10000 ELM327_Emulator.ino.bin
```

## Testen

`tools/elm_test.py` schickt eine Reihe typischer Befehle über USB und zeigt die Antworten:

```bash
pip install pyserial
python tools/elm_test.py COM3
```

Über Bluetooth funktioniert jede OBD-App (z.B. Car Scanner, Torque) – Adapter „OBDII“ wählen.

## Lizenz

MIT, siehe [LICENSE](LICENSE).
