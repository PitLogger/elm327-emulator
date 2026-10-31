/*
 * ELM327-Emulator für ESP32
 * ---------------------------------------------------------------
 * Der ESP32 verhält sich über die USB-Seriell-Schnittstelle (z.B. COM3)
 * wie ein ELM327 OBD-II-Adapter, der an einem Fahrzeug mit
 * ISO 15765-4 CAN (11 bit, 500 kbit/s) hängt.
 *
 * Unterstützt:
 *   - AT-Befehle: Z, WS, D, I, @1, E0/1, L0/1, S0/1, H0/1, SP/TP, DP, DPN,
 *                 RV, MA (Monitor All) und viele weitere werden mit OK quittiert
 *   - Mode 01: Live-Daten (simuliertes Fahrprofil), mehrere PIDs pro Anfrage
 *   - Mode 03/07/0A: Fehlercodes, Mode 04: Fehler löschen
 *   - Mode 09: VIN (Multi-Frame / ISO-TP) und ECU-Name
 *   - Antworten mit/ohne Header (ATH1/ATH0) und mit/ohne Leerzeichen (ATS0/1)
 *
 * Erreichbar über drei Wege (gleichzeitig nutzbar):
 *   - USB-Seriell (z.B. COM3), 38400 Baud, 8N1, Zeilenende CR
 *   - Bluetooth Classic SPP unter dem Namen "OBDII" (wie günstige ELM327-BT-Adapter)
 *   - Bluetooth Low Energy unter dem Namen "OBDII" (wie Vgate/Veepeak-BLE-Adapter):
 *     Service FFF0, Antworten per Notify auf FFF1, Befehle per Write auf FFF2
 *
 * Bluetooth-Schalter: Bluetooth ist standardmäßig an. GPIO 13 mit GND brücken
 * macht den Emulator unsichtbar (trennt Verbindungen, keine Werbung/Sichtbarkeit).
 * Boards ohne herausgeführten GPIO 13 haben dadurch immer Bluetooth (interner
 * Pull-up). Umschalten geht im laufenden Betrieb, die Onboard-LED (GPIO 2) zeigt
 * den Zustand an.
 *
 * Board: ESP32 mit Bluetooth Classic (das ursprüngliche ESP32, nicht S2/S3/C3).
 * Classic + BLE passen nicht mehr ins Standard-Partitionsschema:
 *   arduino-cli compile --upload -p COM3 --fqbn esp32:esp32:esp32:PartitionScheme=huge_app ELM327_Emulator
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_gap_bt_api.h>
#include <freertos/stream_buffer.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic (SPP) ist für dieses Board nicht verfügbar"
#endif

// ---------------- Konfiguration ----------------
#define BT_NAME         "OBDII"               // Gerätename, wie ihn die meisten ELM327-Adapter melden
#define BT_PIN          "1234"                // Kopplungs-PIN (Legacy Pairing) wie bei echten Adaptern
#define BT_SWITCH_PIN   13                    // mit GND gebrückt = Bluetooth aus
#define BT_LED_PIN      2                     // LED leuchtet bei aktivem Bluetooth, -1 = keine LED
#define BT_DEBOUNCE_MS  50
#define BLE_SERVICE     0xFFF0                // wie viele BLE-ELM327 (Vgate, Veepeak, Clones)
#define BLE_NOTIFY      0xFFF1                // Adapter -> App
#define BLE_WRITE       0xFFF2                // App -> Adapter
#define BLE_CHUNK       20                    // Nutzdaten pro Notify bei Standard-MTU
#define ELM_BAUD        38400                 // Standard-Baudrate eines ELM327
#define ELM_VERSION     "ELM327 v1.5"
#define ELM_DEVICE_DESC "OBDII to RS232 Interpreter"
#define ECU_HEADER      "7E8"                 // CAN-ID der Antworten des Motorsteuergeräts

static const char VIN[]      = "WVWZZZ1KZ6W000001";   // genau 17 Zeichen
static const char ECU_NAME[] = "ECM-EngineControl";   // max. 20 Zeichen

// ---------------- ELM-Zustand ----------------
struct ElmSettings {
  bool    echo;
  bool    linefeed;
  bool    spaces;
  bool    headers;
  bool    autoProtocol;
  uint8_t protocol;
};

// Jede Schnittstelle hat ihren eigenen Eingabepuffer; geantwortet wird immer
// auf der Schnittstelle, von der der aktuelle Befehl kam.
struct Port {
  Stream *stream;
  char    lineBuf[64];
  size_t  lineLen;
  bool    lineOverflow;
};

// Stream über BLE-GATT: Writes auf FFF2 landen im Empfangspuffer, Ausgaben werden
// gesammelt und spätestens beim '>'-Prompt als Notify auf FFF1 verschickt.
class BleStream : public Stream {
public:
  void begin(BLECharacteristic *notifyChar) {
    tx = notifyChar;
    rx = xStreamBufferCreate(256, 1);
  }
  void feed(const uint8_t *d, size_t n) { xStreamBufferSend(rx, d, n, 0); }
  void setConnected(bool c) {
    connected = c;
    txLen = 0;
  }
  bool isConnected() const { return connected; }

  int available() override { return rx ? xStreamBufferBytesAvailable(rx) : 0; }
  int read() override {
    uint8_t c;
    return rx && xStreamBufferReceive(rx, &c, 1, 0) == 1 ? c : -1;
  }
  int peek() override { return -1; }   // wird vom Emulator nicht benutzt
  size_t write(uint8_t c) override {
    if (!connected) return 1;
    txBuf[txLen++] = c;
    if (c == '>' || txLen == BLE_CHUNK) flush();
    return 1;
  }
  void flush() override {
    if (txLen && connected) {
      tx->setValue(txBuf, txLen);
      tx->notify();
    }
    txLen = 0;
  }

private:
  BLECharacteristic   *tx = nullptr;
  StreamBufferHandle_t rx = nullptr;
  uint8_t              txBuf[BLE_CHUNK];
  size_t               txLen = 0;
  volatile bool        connected = false;
};

static BluetoothSerial SerialBT;
static BleStream       SerialBLE;
static BLEServer      *bleServer;
static Port ports[] = {
  { &Serial,    {}, 0, false },
  { &SerialBT,  {}, 0, false },
  { &SerialBLE, {}, 0, false },
};
static Stream *io = &Serial;
static bool    btActive = false;

static ElmSettings cfg;
static String lastCmd;

static const char *PROTOCOL_NAMES[] = {
  "AUTO",
  "SAE J1850 PWM",
  "SAE J1850 VPW",
  "ISO 9141-2",
  "ISO 14230-4 (KWP 5BAUD)",
  "ISO 14230-4 (KWP FAST)",
  "ISO 15765-4 (CAN 11/500)",
  "ISO 15765-4 (CAN 29/500)",
  "ISO 15765-4 (CAN 11/250)",
  "ISO 15765-4 (CAN 29/250)",
  "SAE J1939 (CAN 29/250)",
  "USER1 CAN (11* /125)",
  "USER2 CAN (11* /50)",
};

static void resetSettings() {
  cfg.echo         = true;
  cfg.linefeed     = false;
  cfg.spaces       = true;
  cfg.headers      = false;
  cfg.autoProtocol = true;
  cfg.protocol     = 6;
}

// ---------------- Simuliertes Fahrzeug ----------------
struct Vehicle {
  float rpm, speed, throttle, load, maf, mapKpa, timing;
  float coolant, oil, intake, ambient, baro;
  float fuelLevel, voltage, distMil;
  uint32_t startMs;
};

static Vehicle car;
static uint16_t dtcs[8] = { 0x0133, 0x0301 };   // P0133, P0301
static uint8_t  dtcCount = 2;

static void initVehicle() {
  car = {};
  car.rpm       = 800;
  car.coolant   = 20;
  car.oil       = 20;
  car.ambient   = 18;
  car.intake    = 20;
  car.baro      = 101;
  car.fuelLevel = 65;
  car.voltage   = 12.6;
  car.startMs   = millis();
}

// Einfaches Fahrprofil: Geschwindigkeit pendelt alle 90 s zwischen 0 und 120 km/h,
// Motor wird langsam warm, Tank leert sich.
static void updateVehicle() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < 100) return;
  float dt = (now - last) / 1000.0f;
  last = now;

  float t     = (now - car.startMs) / 1000.0f;
  float phase = 2.0f * PI * t / 90.0f;

  car.speed    = max(0.0f, 60.0f - 60.0f * cosf(phase));
  car.throttle = constrain(12.0f + car.speed * 0.3f + 35.0f * sinf(phase), 0.0f, 100.0f);
  car.rpm      = 800.0f + car.speed * 28.0f + random(-25, 26);
  car.load     = constrain(18.0f + car.throttle * 0.75f, 0.0f, 100.0f);
  car.maf      = car.rpm * car.load / 5000.0f;
  car.mapKpa   = 30.0f + car.load * 0.7f;
  car.timing   = 8.0f + car.speed / 10.0f;

  car.coolant += (90.0f - car.coolant) * dt / 60.0f;
  car.oil     += (95.0f - car.oil) * dt / 120.0f;
  car.intake   = car.ambient + 8.0f + car.throttle * 0.05f;
  car.fuelLevel = max(0.0f, car.fuelLevel - 0.005f * dt);
  car.voltage  = 14.1f + random(-10, 11) / 100.0f;

  if (dtcCount > 0) car.distMil += car.speed * dt / 3600.0f;
}

// ---------------- Ausgabe-Hilfen ----------------
static void eol() {
  io->print('\r');
  if (cfg.linefeed) io->print('\n');
}

static void printLine(const String &s) {
  io->print(s);
  eol();
}

static void appendByte(String &out, uint8_t b) {
  char hex[3];
  snprintf(hex, sizeof(hex), "%02X", b);
  if (cfg.spaces && out.length() > 0) out += ' ';
  out += hex;
}

// Gibt eine ECU-Antwort aus. Bis 7 Byte als Single Frame, darüber als
// ISO-TP Multi-Frame im typischen ELM327-Format.
static void sendEcuResponse(const uint8_t *data, size_t len) {
  if (len <= 7) {
    String line;
    if (cfg.headers) {
      line = ECU_HEADER;
      appendByte(line, len);
    }
    for (size_t i = 0; i < len; i++) appendByte(line, data[i]);
    printLine(line);
    return;
  }

  if (!cfg.headers) {
    char total[5];
    snprintf(total, sizeof(total), "%03X", (unsigned)len);
    printLine(total);
  }

  size_t  pos = 0;
  uint8_t seq = 0;
  while (pos < len) {
    String line;
    size_t chunk = (seq == 0) ? 6 : 7;
    if (cfg.headers) {
      line = ECU_HEADER;
      if (seq == 0) {
        appendByte(line, 0x10 | ((len >> 8) & 0x0F));
        appendByte(line, len & 0xFF);
      } else {
        appendByte(line, 0x20 | (seq & 0x0F));
      }
    } else {
      line = String(seq & 0x0F, HEX);
      line.toUpperCase();
      line += ':';
    }
    for (size_t i = 0; i < chunk && pos < len; i++) appendByte(line, data[pos++]);
    printLine(line);
    seq++;
  }
}

// ---------------- Mode 01 ----------------
static const uint8_t SUPPORTED_01[] = {
  0x01, 0x04, 0x05, 0x06, 0x07, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
  0x1C, 0x1F, 0x21, 0x2F, 0x33, 0x42, 0x46, 0x51, 0x5C,
};

static bool isSupported01(uint8_t pid) {
  for (uint8_t p : SUPPORTED_01) if (p == pid) return true;
  return false;
}

static bool anySupportedAbove(uint8_t pid) {
  for (uint8_t p : SUPPORTED_01) if (p > pid) return true;
  return false;
}

static uint32_t supportBitmask(uint8_t base) {
  uint32_t mask = 0;
  for (int i = 1; i <= 32; i++) {
    int pid = base + i;
    if (pid > 0xFF) break;
    bool s = isSupported01(pid) || (i == 32 && anySupportedAbove(pid));
    if (s) mask |= 1UL << (32 - i);
  }
  return mask;
}

static uint8_t pct(float v)  { return (uint8_t)constrain(lroundf(v * 255.0f / 100.0f), 0L, 255L); }
static uint8_t temp(float v) { return (uint8_t)constrain(lroundf(v + 40.0f), 0L, 255L); }

// Schreibt die Datenbytes eines PIDs nach out, liefert die Anzahl oder -1.
static int readPid01(uint8_t pid, uint8_t *out) {
  if ((pid & 0x1F) == 0) {
    if (pid != 0 && !anySupportedAbove(pid)) return -1;
    uint32_t m = supportBitmask(pid);
    out[0] = m >> 24; out[1] = m >> 16; out[2] = m >> 8; out[3] = m;
    return 4;
  }
  if (!isSupported01(pid)) return -1;

  uint16_t v;
  switch (pid) {
    case 0x01:  // Monitorstatus: MIL + Anzahl DTCs
      out[0] = (dtcCount ? 0x80 : 0x00) | (dtcCount & 0x7F);
      out[1] = 0x07; out[2] = 0xE5; out[3] = 0x00;
      return 4;
    case 0x04: out[0] = pct(car.load); return 1;
    case 0x05: out[0] = temp(car.coolant); return 1;
    case 0x06:                              // Kurzzeit-Kraftstofftrimm Bank 1
    case 0x07: out[0] = 0x80 + random(-3, 4); return 1;
    case 0x0B: out[0] = (uint8_t)car.mapKpa; return 1;
    case 0x0C:
      v = (uint16_t)(car.rpm * 4.0f);
      out[0] = v >> 8; out[1] = v;
      return 2;
    case 0x0D: out[0] = (uint8_t)car.speed; return 1;
    case 0x0E: out[0] = (uint8_t)constrain((car.timing + 64.0f) * 2.0f, 0.0f, 255.0f); return 1;
    case 0x0F: out[0] = temp(car.intake); return 1;
    case 0x10:
      v = (uint16_t)(car.maf * 100.0f);
      out[0] = v >> 8; out[1] = v;
      return 2;
    case 0x11: out[0] = pct(car.throttle); return 1;
    case 0x1C: out[0] = 0x06; return 1;     // EOBD
    case 0x1F:
      v = (uint16_t)min<uint32_t>((millis() - car.startMs) / 1000, 0xFFFF);
      out[0] = v >> 8; out[1] = v;
      return 2;
    case 0x21:
      v = (uint16_t)car.distMil;
      out[0] = v >> 8; out[1] = v;
      return 2;
    case 0x2F: out[0] = pct(car.fuelLevel); return 1;
    case 0x33: out[0] = (uint8_t)car.baro; return 1;
    case 0x42:
      v = (uint16_t)(car.voltage * 1000.0f);
      out[0] = v >> 8; out[1] = v;
      return 2;
    case 0x46: out[0] = temp(car.ambient); return 1;
    case 0x51: out[0] = 0x01; return 1;     // Benzin
    case 0x5C: out[0] = temp(car.oil); return 1;
  }
  return -1;
}

// ---------------- OBD-Anfragen ----------------
static void sendNegative(uint8_t mode, uint8_t nrc) {
  uint8_t r[] = { 0x7F, mode, nrc };
  sendEcuResponse(r, sizeof(r));
}

static void sendDtcList(uint8_t respMode, const uint16_t *list, uint8_t count) {
  uint8_t r[2 + 2 * 8];
  size_t n = 0;
  r[n++] = respMode;
  r[n++] = count;
  for (uint8_t i = 0; i < count; i++) {
    r[n++] = list[i] >> 8;
    r[n++] = list[i] & 0xFF;
  }
  sendEcuResponse(r, n);
}

static void handleMode09(uint8_t pid) {
  uint8_t r[32];
  size_t n = 0;
  r[n++] = 0x49;
  r[n++] = pid;
  switch (pid) {
    case 0x00: {
      uint32_t m = (1UL << (32 - 0x02)) | (1UL << (32 - 0x0A));
      r[n++] = m >> 24; r[n++] = m >> 16; r[n++] = m >> 8; r[n++] = m;
      break;
    }
    case 0x02:
      r[n++] = 0x01;
      for (size_t i = 0; i < 17; i++) r[n++] = VIN[i];
      break;
    case 0x0A:
      r[n++] = 0x01;
      for (size_t i = 0; i < 20; i++) r[n++] = i < strlen(ECU_NAME) ? ECU_NAME[i] : 0x00;
      break;
    default:
      printLine("NO DATA");
      return;
  }
  sendEcuResponse(r, n);
}

static void handleObd(String cmd) {
  for (size_t i = 0; i < cmd.length(); i++) {
    if (!isxdigit(cmd[i])) { printLine("?"); return; }
  }
  // Ungerade Länge: letzte Ziffer ist die erwartete Antwortanzahl (z.B. "010C1")
  if (cmd.length() % 2 == 1) {
    if (cmd.length() < 3) { printLine("?"); return; }
    cmd.remove(cmd.length() - 1);
  }

  uint8_t req[8];
  size_t  n = cmd.length() / 2;
  if (n > sizeof(req)) { printLine("?"); return; }
  for (size_t i = 0; i < n; i++) req[i] = strtoul(cmd.substring(2 * i, 2 * i + 2).c_str(), nullptr, 16);

  uint8_t mode = req[0];
  switch (mode) {
    case 0x01: {
      uint8_t resp[64];
      size_t  len = 0;
      resp[len++] = 0x41;
      for (size_t i = 1; i < n && i <= 6; i++) {
        uint8_t data[4];
        int dl = readPid01(req[i], data);
        if (dl < 0) continue;
        resp[len++] = req[i];
        memcpy(resp + len, data, dl);
        len += dl;
      }
      if (len == 1) printLine("NO DATA");
      else sendEcuResponse(resp, len);
      break;
    }
    case 0x03: sendDtcList(0x43, dtcs, dtcCount); break;
    case 0x07: sendDtcList(0x47, nullptr, 0); break;
    case 0x0A: sendDtcList(0x4A, nullptr, 0); break;
    case 0x04: {
      dtcCount = 0;
      car.distMil = 0;
      uint8_t r[] = { 0x44 };
      sendEcuResponse(r, 1);
      break;
    }
    case 0x09:
      if (n < 2) printLine("NO DATA");
      else handleMode09(req[1]);
      break;
    case 0x02:  // Freeze Frame nicht implementiert
      printLine("NO DATA");
      break;
    default:
      sendNegative(mode, 0x11);  // serviceNotSupported
      break;
  }
}

// ---------------- AT-Befehle ----------------
static bool onOff(const String &a, const char *prefix, bool &target) {
  if (a == String(prefix) + "0") { target = false; return true; }
  if (a == String(prefix) + "1") { target = true;  return true; }
  return false;
}

static String protocolName() {
  uint8_t p = cfg.protocol < 13 ? cfg.protocol : 0;
  return cfg.autoProtocol ? String("AUTO, ") + PROTOCOL_NAMES[p] : String(PROTOCOL_NAMES[p]);
}

static bool setProtocol(String arg) {
  bool autoP = false;
  if (arg.startsWith("A")) { autoP = true; arg.remove(0, 1); }
  if (arg.length() != 1 || !isxdigit(arg[0])) return false;
  uint8_t p = strtoul(arg.c_str(), nullptr, 16);
  if (p > 0x0C) return false;
  if (p == 0) { cfg.autoProtocol = true; cfg.protocol = 6; }
  else        { cfg.autoProtocol = autoP; cfg.protocol = p; }
  return true;
}

// ATMA: gibt simulierten CAN-Busverkehr aus, bis ein Zeichen empfangen wird.
static void monitorAll() {
  uint8_t idx = 0;
  while (!io->available()) {
    updateVehicle();
    String line;
    uint8_t d[8] = {};
    const char *id;
    switch (idx++ % 3) {
      case 0: {
        id = "0C9";
        uint16_t r = (uint16_t)(car.rpm * 4.0f);
        d[1] = r >> 8; d[2] = r; d[4] = pct(car.throttle);
        break;
      }
      case 1: {
        id = "3E9";
        uint16_t s = (uint16_t)(car.speed * 100.0f);
        d[0] = s >> 8; d[1] = s;
        break;
      }
      default:
        id = "4C1";
        d[2] = temp(car.coolant); d[3] = temp(car.oil); d[5] = pct(car.fuelLevel);
        break;
    }
    line = id;
    for (uint8_t b : d) appendByte(line, b);
    printLine(line);
    delay(20);
  }
  io->read();  // Abbruchzeichen verwerfen
}

static void handleAt(const String &a) {
  if (a == "Z" || a == "WS") {
    delay(a == "Z" ? 500 : 100);
    resetSettings();
    eol(); eol();
    printLine(ELM_VERSION);
    return;
  }
  if (a == "I")   { printLine(ELM_VERSION); return; }
  if (a == "@1")  { printLine(ELM_DEVICE_DESC); return; }
  if (a == "D")   { resetSettings(); printLine("OK"); return; }
  if (a == "DP")  { printLine(protocolName()); return; }
  if (a == "DPN") {
    char s[4];
    snprintf(s, sizeof(s), cfg.autoProtocol ? "A%X" : "%X", cfg.protocol);
    printLine(s);
    return;
  }
  if (a == "RV") {
    char s[8];
    snprintf(s, sizeof(s), "%.1fV", car.rpm > 500 ? car.voltage : 12.6f);
    printLine(s);
    return;
  }
  if (a == "MA") { monitorAll(); return; }

  if (onOff(a, "E", cfg.echo) || onOff(a, "L", cfg.linefeed) ||
      onOff(a, "S", cfg.spaces) || onOff(a, "H", cfg.headers)) {
    printLine("OK");
    return;
  }

  if (a.startsWith("SP") || a.startsWith("TP")) {
    printLine(setProtocol(a.substring(2)) ? "OK" : "?");
    return;
  }

  // Befehle, die nur quittiert werden (Timing, Filter, Header, ...)
  static const char *ACK_PREFIXES[] = {
    "M", "AT", "CAF", "CFC", "AL", "NL", "ST", "SH", "CRA", "CF", "CM", "FC",
    "PC", "SW", "IB", "IIA", "KW", "BI", "V", "R", "CEA", "SS", "D0", "D1",
  };
  for (const char *p : ACK_PREFIXES) {
    if (a.startsWith(p)) { printLine("OK"); return; }
  }

  printLine("?");
}

// ---------------- Befehlsverarbeitung ----------------
static void processLine(Port &port) {
  String cmd;
  if (port.lineOverflow) {
    printLine("?");
  } else {
    cmd = String(port.lineBuf).substring(0, port.lineLen);
    if (cmd.length() == 0) cmd = lastCmd;   // leere Zeile wiederholt letzten Befehl
    else lastCmd = cmd;

    if (cmd.startsWith("AT")) handleAt(cmd.substring(2));
    else if (cmd.length() > 0) handleObd(cmd);
  }
  eol();
  io->print('>');
}

static void pollPort(Port &port) {
  while (port.stream->available()) {
    io = port.stream;
    char c = port.stream->read();

    if (cfg.echo && c != '\n') {
      io->print(c);
      if (c == '\r' && cfg.linefeed) io->print('\n');
    }

    if (c == '\r') {
      port.lineBuf[port.lineLen] = '\0';
      processLine(port);
      port.lineLen = 0;
      port.lineOverflow = false;
    } else if (c == ' ' || c == '\n' || c == '\t' || c == '\0') {
      continue;
    } else if (port.lineLen < sizeof(port.lineBuf) - 1) {
      port.lineBuf[port.lineLen++] = toupper(c);
    } else {
      port.lineOverflow = true;
    }
  }
}

// ---------------- Bluetooth (Classic + BLE) ----------------
class BleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { SerialBLE.setConnected(true); }
  void onDisconnect(BLEServer *) override {
    SerialBLE.setConnected(false);
    if (btActive) BLEDevice::startAdvertising();
  }
};

class BleWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    String v = c->getValue();
    SerialBLE.feed((const uint8_t *)v.c_str(), v.length());
  }
};

// Beide Stacks laufen dauerhaft; SerialBT.end() würde Bluedroid und damit auch BLE abschalten.
static void initBluetooth() {
  SerialBT.begin(BT_NAME);
  SerialBT.setPin(BT_PIN, strlen(BT_PIN));

  BLEDevice::init(BT_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());
  BLEService *svc = bleServer->createService(BLEUUID((uint16_t)BLE_SERVICE));
  BLECharacteristic *notifyChar = svc->createCharacteristic(
    BLEUUID((uint16_t)BLE_NOTIFY), BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  notifyChar->addDescriptor(new BLE2902());
  BLECharacteristic *writeChar = svc->createCharacteristic(
    BLEUUID((uint16_t)BLE_WRITE), BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  writeChar->setCallbacks(new BleWriteCallbacks());
  svc->start();
  SerialBLE.begin(notifyChar);

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID((uint16_t)BLE_SERVICE));
  adv->setScanResponse(true);   // Name steht in der Scan-Antwort
  BLEDevice::startAdvertising();
  btActive = true;              // nach dem Start sind beide sichtbar
}

// Schaltet die Sichtbarkeit beider Bluetooth-Varianten um und trennt bestehende Verbindungen.
static void setBluetooth(bool on) {
  if (on == btActive) return;
  btActive = on;
  if (on) {
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    BLEDevice::startAdvertising();
  } else {
    if (io != &Serial) io = &Serial;
    SerialBT.disconnect();
    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    BLEDevice::stopAdvertising();
    if (SerialBLE.isConnected()) bleServer->disconnect(bleServer->getConnId());
    for (int i = 1; i <= 2; i++) {
      ports[i].lineLen = 0;
      ports[i].lineOverflow = false;
    }
  }
#if BT_LED_PIN >= 0
  digitalWrite(BT_LED_PIN, on ? HIGH : LOW);
#endif
}

// Offener Pin (interner Pull-up) = Bluetooth an, gegen GND gebrückt = aus.
static bool btWanted() {
  return digitalRead(BT_SWITCH_PIN) == HIGH;
}

// Entprellt: der neue Zustand gilt erst, wenn er BT_DEBOUNCE_MS stabil anliegt.
static void checkBtSwitch() {
  static bool     lastRaw   = btWanted();
  static uint32_t changedAt = 0;
  bool raw = btWanted();
  if (raw != lastRaw) {
    lastRaw   = raw;
    changedAt = millis();
  } else if (raw != btActive && millis() - changedAt >= BT_DEBOUNCE_MS) {
    setBluetooth(raw);
  }
}

void setup() {
  Serial.begin(ELM_BAUD);
  resetSettings();
  initVehicle();
  randomSeed(esp_random());

  pinMode(BT_SWITCH_PIN, INPUT_PULLUP);
#if BT_LED_PIN >= 0
  pinMode(BT_LED_PIN, OUTPUT);
  digitalWrite(BT_LED_PIN, LOW);
#endif
  delay(10);
  initBluetooth();
  setBluetooth(btWanted());

  delay(200);
  io = &Serial;
  printLine(ELM_VERSION);
  eol();
  io->print('>');
}

void loop() {
  updateVehicle();
  checkBtSwitch();
  pollPort(ports[0]);
  if (btActive) {
    pollPort(ports[1]);
    pollPort(ports[2]);
  }
}
