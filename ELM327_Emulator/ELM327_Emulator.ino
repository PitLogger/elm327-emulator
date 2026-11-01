/*
 * ELM327 emulator for ESP32
 * ---------------------------------------------------------------
 * The ESP32 behaves like an ELM327 OBD-II adapter connected to a vehicle with
 * ISO 15765-4 CAN (11 bit, 500 kbit/s).
 *
 * Supported:
 *   - AT commands: Z, WS, D, I, @1, E0/1, L0/1, S0/1, H0/1, SP/TP, DP, DPN,
 *                  RV, MA (monitor all); many others are acknowledged with OK
 *   - mode 01: live data (simulated drive profile), several PIDs per request
 *   - mode 03/07/0A: trouble codes, mode 04: clear trouble codes
 *   - mode 09: VIN (multi-frame / ISO-TP) and ECU name
 *   - responses with/without headers (ATH1/ATH0) and with/without spaces (ATS0/1)
 *   - manufacturer requests (services 21/22) of real vehicles with their recorded
 *     answers (vehicles.h), selected with ATSH like on a real adapter
 *
 * CAN bus: with an SN65HVD230 transceiver on GPIO 21 (TX) and 22 (RX) the emulator is also a
 * vehicle on a real 500 kbit/s CAN bus: it answers OBD-II requests (7DF / 7E0 -> 7E8) and the
 * recorded manufacturer requests of the selected vehicle, with ISO-TP flow control.
 *
 * "ATPLC" shows the state of the CAN controller. "ATPLB300" makes the recorded control units
 * answer "busy" (7F xx 21) to a request repeated within 300 ms ("ATPLB0" = off, not stored).
 * "ATPLN1" hides the VIN like some electric vehicles do ("ATPLN0" = report it, not stored).
 * Firmware updates over the same USB port without BOOT/EN: python tools/serial_update.py COM8 file.bin
 * Vehicle: "ATPLV" lists the vehicles, "ATPLV1" selects one (stored across restarts).
 *   0 = simulated VW Golf (only standard OBD-II), 1 = Hyundai i30 N 2021, 2 = Toyota bZ4X 2022
 *
 * Reachable in three ways (usable at the same time):
 *   - USB serial (e.g. COM3), 38400 baud, 8N1, line ending CR
 *   - Bluetooth Classic SPP named "OBDII" (like cheap ELM327 BT adapters)
 *   - Bluetooth Low Energy named "OBDII" (like Vgate/Veepeak BLE adapters):
 *     service FFF0, responses via notify on FFF1, commands via write on FFF2
 *
 * Bluetooth switch: Bluetooth is on by default. Connecting GPIO 13 to GND makes the
 * emulator invisible (drops connections, no advertising / discoverability). Boards
 * without GPIO 13 broken out therefore always have Bluetooth (internal pull-up).
 * Switching works at runtime; the onboard LED (GPIO 2) shows the state.
 *
 * Board: ESP32 with Bluetooth Classic (the original ESP32, not S2/S3/C3).
 * Classic + BLE no longer fit into the default partition scheme:
 *   arduino-cli compile --upload -p COM3 --fqbn esp32:esp32:esp32:PartitionScheme=huge_app ELM327_Emulator
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <esp_gap_bt_api.h>
#include <freertos/stream_buffer.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <driver/twai.h>
#include "vehicles.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic (SPP) is not available on this board"
#endif

// ---------------- Configuration ----------------
#define BT_NAME         "OBDII"               // device name reported by most ELM327 adapters
#define BT_PIN          "1234"                // pairing PIN (legacy pairing) as on real adapters
#define BT_SWITCH_PIN   13                    // connected to GND = Bluetooth off
#define BT_LED_PIN      2                     // LED on while Bluetooth is active, -1 = no LED
#define BT_DEBOUNCE_MS  50
#define BLE_SERVICE     0xFFF0                // like many BLE ELM327 (Vgate, Veepeak, clones)
#define BLE_NOTIFY      0xFFF1                // Adapter -> App
#define BLE_WRITE       0xFFF2                // App -> Adapter
#define BLE_CHUNK       20                    // payload per notification at the default MTU
#define ELM_BAUD        38400                 // default baud rate of an ELM327
#define ELM_VERSION     "ELM327 v1.5"
#define ELM_DEVICE_DESC "OBDII to RS232 Interpreter"
#define ECU_REQUEST     0x7E0                 // CAN ID of requests to the engine control unit
#define ECU_RESPONSE    0x7E8                 // CAN ID of its responses
#define OBD_FUNCTIONAL  0x7DF                 // CAN ID of requests to all control units
#define CAN_TX_PIN      21                    // to CTX of the SN65HVD230
#define CAN_RX_PIN      22                    // to CRX of the SN65HVD230
#define CAN_PADDING     0xAA                  // fills unused bytes of a frame
#define CAN_FC_WAIT_MS  1000                  // wait for the tester's flow control frame
#define UPDATE_BAUD     460800                // firmware update over USB (ATPLUPD)
#define UPDATE_BLOCK    1024                  // bytes per acknowledged block
#define UPDATE_TIMEOUT  5000                  // ms without data before the update is abandoned

static const char SIM_VIN[]  = "WVWZZZ1KZ6W000001";   // exactly 17 characters
static const char ECU_NAME[] = "ECM-EngineControl";   // max. 20 characters

// ---------------- ELM state ----------------
struct ElmSettings {
  bool    echo;
  bool    linefeed;
  bool    spaces;
  bool    headers;
  bool    autoProtocol;
  uint8_t protocol;
};

// Every interface has its own input buffer; the response always goes to
// the interface the current command came from.
struct Port {
  Stream *stream;
  char    lineBuf[64];
  size_t  lineLen;
  bool    lineOverflow;
};

// Stream over BLE GATT: writes to FFF2 go into the receive buffer, output is collected
// and sent as a notification on FFF1 at the latest with the '>' prompt.
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
  int peek() override { return -1; }   // not used by the emulator
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
static uint16_t elmHeader = OBD_FUNCTIONAL;   // set with ATSH

// Selected vehicle: 0 = simulation only, 1.. = REC_VEHICLES[n - 1]
static Preferences prefs;
static uint8_t     vehicleSel;
static uint16_t    busyMs;
static bool        hideVin;  // ATPLN1: behave like vehicles that do not report their VIN   // ATPLB: a recorded request asked again within this time gets "busy" (0 = off)

static const RecVehicle *recVehicle() {
  return vehicleSel >= 1 && vehicleSel <= REC_VEHICLE_COUNT ? &REC_VEHICLES[vehicleSel - 1] : nullptr;
}

static const char *vin() {
  const RecVehicle *v = recVehicle();
  return v ? v->vin : SIM_VIN;
}

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
  elmHeader        = OBD_FUNCTIONAL;
}

// ---------------- Simulated vehicle ----------------
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

// Simple drive profile: speed swings between 0 and 120 km/h every 90 s,
// the engine slowly warms up, the tank empties.
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

// ---------------- Output helpers ----------------
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

static String canIdText(uint16_t id) {
  char s[4];
  snprintf(s, sizeof(s), "%03X", id);
  return s;
}

// Outputs an ECU response. Up to 7 bytes as a single frame, above that as an
// ISO-TP multi-frame in the typical ELM327 format.
static void sendEcuResponse(const uint8_t *data, size_t len, uint16_t from = ECU_RESPONSE) {
  if (len <= 7) {
    String line;
    if (cfg.headers) {
      line = canIdText(from);
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
      line = canIdText(from);
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

// Writes the data bytes of a PID to out, returns the count or -1.
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
    case 0x01:  // monitor status: MIL + number of DTCs
      out[0] = (dtcCount ? 0x80 : 0x00) | (dtcCount & 0x7F);
      out[1] = 0x07; out[2] = 0xE5; out[3] = 0x00;
      return 4;
    case 0x04: out[0] = pct(car.load); return 1;
    case 0x05: out[0] = temp(car.coolant); return 1;
    case 0x06:                              // short-term fuel trim bank 1
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
    case 0x51: out[0] = 0x01; return 1;     // gasoline
    case 0x5C: out[0] = temp(car.oil); return 1;
  }
  return -1;
}

// ---------------- OBD requests ----------------
// The answers are built as bytes and then output either as ELM327 text or as CAN frames.

static size_t dtcList(uint8_t respMode, const uint16_t *list, uint8_t count, uint8_t *r) {
  size_t n = 0;
  r[n++] = respMode;
  r[n++] = count;
  for (uint8_t i = 0; i < count; i++) {
    r[n++] = list[i] >> 8;
    r[n++] = list[i] & 0xFF;
  }
  return n;
}

static size_t mode09(uint8_t pid, uint8_t *r) {
  size_t n = 0;
  r[n++] = 0x49;
  r[n++] = pid;
  switch (pid) {
    case 0x00: {
      uint32_t m = (hideVin ? 0 : 1UL << (32 - 0x02)) | (1UL << (32 - 0x0A));
      r[n++] = m >> 24; r[n++] = m >> 16; r[n++] = m >> 8; r[n++] = m;
      return n;
    }
    case 0x02:
      if (hideVin) return 0;
      r[n++] = 0x01;
      for (size_t i = 0; i < 17; i++) r[n++] = vin()[i];
      return n;
    case 0x0A:
      r[n++] = 0x01;
      for (size_t i = 0; i < 20; i++) r[n++] = i < strlen(ECU_NAME) ? ECU_NAME[i] : 0x00;
      return n;
  }
  return 0;
}

// Answer of the simulated engine control unit to a standard OBD-II request (service first).
// Returns the length, 0 = no answer ("NO DATA").
static size_t ecuAnswer(const uint8_t *req, size_t n, uint8_t *resp) {
  if (n == 0) return 0;
  switch (req[0]) {
    case 0x01: {
      size_t len = 0;
      resp[len++] = 0x41;
      for (size_t i = 1; i < n && i <= 6; i++) {
        uint8_t data[4];
        int dl = readPid01(req[i], data);
        if (dl < 0) continue;
        resp[len++] = req[i];
        memcpy(resp + len, data, dl);
        len += dl;
      }
      return len == 1 ? 0 : len;
    }
    case 0x03: return dtcList(0x43, dtcs, dtcCount, resp);
    case 0x07: return dtcList(0x47, nullptr, 0, resp);
    case 0x0A: return dtcList(0x4A, nullptr, 0, resp);
    case 0x04:
      dtcCount    = 0;
      car.distMil = 0;
      resp[0] = 0x44;
      return 1;
    case 0x09: return n < 2 ? 0 : mode09(req[1], resp);
    case 0x02: return 0;   // freeze frame not implemented
  }
  resp[0] = 0x7F; resp[1] = req[0]; resp[2] = 0x11;   // serviceNotSupported
  return 3;
}

// Recorded answer of the selected real vehicle to a manufacturer request sent to CAN ID tx.
// Consecutive requests return the recorded answers in turn, so values change. A known control
// unit answers unknown identifiers with "request out of range" like a real one.
static bool recordedAnswer(uint16_t tx, const uint8_t *req, size_t n, uint16_t &rx,
                           const uint8_t *&data, size_t &len, uint8_t *negative) {
  const RecVehicle *v = recVehicle();
  if (!v || n == 0) return false;
  static uint8_t  turn[256];
  static uint32_t served[256];   // millis() of the last answer per request
  bool knownUnit = false;
  for (uint16_t i = 0; i < v->requestCount; i++) {
    const RecRequest &r = v->requests[i];
    if (r.tx != tx) continue;
    knownUnit = true;
    rx = r.rx;
    if (r.reqLen != n || memcmp(r.req, req, n) != 0) continue;
    // like an overloaded control unit: too frequent requests get "busy, repeat request"
    if (busyMs && served[i & 0xFF] && millis() - served[i & 0xFF] < busyMs) {
      negative[0] = 0x7F; negative[1] = req[0]; negative[2] = 0x21;
      data = negative;
      len  = 3;
      return true;
    }
    served[i & 0xFF] = millis();
    const RecAnswer &a = REC_ANSWERS[r.first + turn[i & 0xFF]++ % r.count];
    data = REC_DATA + a.offset;
    len  = a.len;
    return true;
  }
  if (!knownUnit || (req[0] != 0x21 && req[0] != 0x22)) return false;
  negative[0] = 0x7F; negative[1] = req[0]; negative[2] = 0x31;   // requestOutOfRange
  data = negative;
  len  = 3;
  return true;
}

// ELM327 text interface: hex request -> answer of the control unit addressed with ATSH
static void handleObd(String cmd) {
  for (size_t i = 0; i < cmd.length(); i++) {
    if (!isxdigit(cmd[i])) { printLine("?"); return; }
  }
  // odd length: the last digit is the expected number of responses (e.g. "010C1")
  if (cmd.length() % 2 == 1) {
    if (cmd.length() < 3) { printLine("?"); return; }
    cmd.remove(cmd.length() - 1);
  }

  uint8_t req[8];
  size_t  n = cmd.length() / 2;
  if (n > sizeof(req)) { printLine("?"); return; }
  for (size_t i = 0; i < n; i++) req[i] = strtoul(cmd.substring(2 * i, 2 * i + 2).c_str(), nullptr, 16);

  uint16_t       rx;
  const uint8_t *rec;
  size_t         len;
  uint8_t        neg[3];
  if (recordedAnswer(elmHeader, req, n, rx, rec, len, neg)) {
    sendEcuResponse(rec, len, rx);
    return;
  }
  if (elmHeader != OBD_FUNCTIONAL && elmHeader != ECU_REQUEST) {
    printLine("NO DATA");
    return;
  }
  uint8_t resp[64];
  len = ecuAnswer(req, n, resp);
  if (len) sendEcuResponse(resp, len);
  else printLine("NO DATA");
}

// ---------------- CAN bus (ISO 15765-2) ----------------
static bool     canUp;
static uint32_t canReceived, canAnswered;   // for ATPLC

static void canBegin() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g.rx_queue_len = 32;
  g.tx_queue_len = 16;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  canUp = twai_driver_install(&g, &t, &f) == ESP_OK && twai_start() == ESP_OK;
}

static bool canFrame(uint16_t id, const uint8_t *d, size_t n) {
  twai_message_t m = {};
  m.identifier       = id;
  m.data_length_code = 8;
  memset(m.data, CAN_PADDING, 8);
  memcpy(m.data, d, n);
  return twai_transmit(&m, pdMS_TO_TICKS(50)) == ESP_OK;
}

// Waits for the tester's flow control frame (sent to fcId). Returns false on timeout or abort.
static bool canWaitFlowControl(uint16_t fcId, uint8_t &blockSize, uint32_t &gapUs) {
  uint32_t start = millis();
  while (millis() - start < CAN_FC_WAIT_MS) {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(10)) != ESP_OK) continue;
    if (m.extd || m.identifier != fcId || m.data_length_code < 3 || (m.data[0] >> 4) != 3) continue;
    uint8_t status = m.data[0] & 0x0F;
    if (status == 1) { start = millis(); continue; }   // wait
    if (status != 0) return false;                      // overflow / abort
    blockSize = m.data[1];
    uint8_t st = m.data[2];
    gapUs = st <= 0x7F ? st * 1000UL : (st >= 0xF1 && st <= 0xF9 ? (st - 0xF0) * 100UL : 127000UL);
    return true;
  }
  return false;
}

// Sends an answer as single frame or as first frame + consecutive frames with flow control.
static void canSend(uint16_t id, uint16_t fcId, const uint8_t *data, size_t len) {
  uint8_t f[8];
  if (len <= 7) {
    f[0] = len;
    memcpy(f + 1, data, len);
    canFrame(id, f, len + 1);
    return;
  }
  f[0] = 0x10 | ((len >> 8) & 0x0F);
  f[1] = len & 0xFF;
  memcpy(f + 2, data, 6);
  if (!canFrame(id, f, 8)) return;
  size_t   pos = 6;
  uint8_t  seq = 1;
  uint8_t  blockSize;
  uint32_t gapUs;
  if (!canWaitFlowControl(fcId, blockSize, gapUs)) return;
  uint8_t inBlock = 0;
  while (pos < len) {
    size_t chunk = min<size_t>(7, len - pos);
    f[0] = 0x20 | (seq++ & 0x0F);
    memcpy(f + 1, data + pos, chunk);
    if (!canFrame(id, f, chunk + 1)) return;
    pos += chunk;
    if (pos >= len) break;
    if (blockSize && ++inBlock >= blockSize) {
      inBlock = 0;
      if (!canWaitFlowControl(fcId, blockSize, gapUs)) return;
    } else if (gapUs) {
      delayMicroseconds(gapUs);
    }
  }
}

// Answers single-frame requests on the bus; restarts the controller after bus-off.
static void canLoop() {
  if (!canUp) return;
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    if (st.state == TWAI_STATE_BUS_OFF) { twai_initiate_recovery(); return; }
    if (st.state == TWAI_STATE_STOPPED) { twai_start(); return; }
  }
  twai_message_t m;
  while (twai_receive(&m, 0) == ESP_OK) {
    canReceived++;
    if (m.extd || m.rtr || m.data_length_code < 2 || (m.data[0] >> 4) != 0) continue;
    size_t n = m.data[0] & 0x0F;
    if (n == 0 || n > 7) continue;
    const uint8_t *req = m.data + 1;
    uint16_t       rx;
    const uint8_t *rec;
    size_t         len;
    uint8_t        neg[3];
    if (recordedAnswer(m.identifier, req, n, rx, rec, len, neg)) {
      canAnswered++;
      canSend(rx, m.identifier, rec, len);
    } else if (m.identifier == OBD_FUNCTIONAL || m.identifier == ECU_REQUEST) {
      updateVehicle();
      uint8_t resp[64];
      len = ecuAnswer(req, n, resp);
      if (len) {
        canAnswered++;
        canSend(ECU_RESPONSE, ECU_REQUEST, resp, len);
      }
    }
  }
}

// ---------------- AT commands ----------------
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

// ATMA: outputs simulated CAN bus traffic until a character is received.
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
  io->read();  // discard the stop character
}

// ---------------- Firmware update over USB ----------------
// "ATPLUPD<size>,<md5>" (tools/serial_update.py): answers "READY", switches to UPDATE_BAUD and
// receives the image in blocks of UPDATE_BLOCK bytes, each acknowledged with 'K'. The image goes
// into the other app partition; with a matching MD5 the emulator answers "DONE" and restarts into
// it, otherwise "ERR <reason>" and it carries on with the old firmware. No BOOT/EN needed.
static void updateFailed(const String &why) {
  Update.abort();
  Serial.print("ERR " + why + "\r");
  Serial.flush();
  delay(50);
  Serial.updateBaudRate(ELM_BAUD);
}

static void serialUpdate(const String &arg) {
  if (io != &Serial) { printLine("? only over USB"); return; }
  int comma = arg.indexOf(',');
  size_t size = arg.substring(0, comma).toInt();
  String md5 = comma > 0 ? arg.substring(comma + 1) : String();
  md5.toLowerCase();
  if (!size || md5.length() != 32) { printLine("?"); return; }
  if (!Update.begin(size)) { printLine(String("ERR ") + Update.errorString()); return; }
  Update.setMD5(md5.c_str());
  Serial.print("READY\r");
  Serial.flush();
  delay(50);
  Serial.updateBaudRate(UPDATE_BAUD);

  static uint8_t buf[UPDATE_BLOCK];
  size_t got = 0;
  while (got < size) {
    size_t want = min<size_t>(UPDATE_BLOCK, size - got), n = 0;
    uint32_t last = millis();
    while (n < want && millis() - last < UPDATE_TIMEOUT) {
      int avail = Serial.available();
      if (avail > 0) {
        n += Serial.readBytes(buf + n, min<size_t>(avail, want - n));
        last = millis();
      } else {
        delay(1);
      }
    }
    if (n < want) return updateFailed("timeout after " + String(got + n) + " bytes");
    if (Update.write(buf, n) != n) return updateFailed(Update.errorString());
    got += n;
    Serial.write('K');
  }
  if (!Update.end()) return updateFailed(Update.errorString());
  Serial.print("DONE\r");
  Serial.flush();
  delay(200);
  ESP.restart();
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

  if (a.startsWith("SH") && a.length() == 5 && isxdigit(a[2]) && isxdigit(a[3]) && isxdigit(a[4])) {
    elmHeader = strtoul(a.c_str() + 2, nullptr, 16);
    printLine("OK");
    return;
  }

  // PitLogger extension: build and app partition of the running firmware
  if (a == "PLI") {
    const esp_partition_t *run = esp_ota_get_running_partition();
    printLine(String("Build ") + __DATE__ + " " + __TIME__ + ", partition " + (run ? run->label : "?"));
    return;
  }

  // PitLogger extension: firmware update over USB, see serialUpdate()
  if (a.startsWith("PLUPD")) { serialUpdate(a.substring(5)); return; }

  // PitLogger extension: state of the CAN controller
  if (a == "PLC") {
    twai_status_info_t st;
    if (!canUp || twai_get_status_info(&st) != ESP_OK) { printLine("CAN off"); return; }
    static const char *STATES[] = { "stopped", "running", "bus-off", "recovering" };
    printLine(String("CAN ") + (st.state < 4 ? STATES[st.state] : "?") + ", TX/RX errors " + st.tx_error_counter +
              "/" + st.rx_error_counter + ", received " + canReceived + ", answered " + canAnswered +
              ", bus errors " + st.bus_error_count);
    return;
  }

  // PitLogger extension: busy control units (ATPLBn = at most one answer per n ms and request, 0 = off)
  if (a.startsWith("PLB")) {
    if (a.length() == 3) { printLine(busyMs ? String("busy below ") + busyMs + " ms" : String("busy mode off")); return; }
    busyMs = a.substring(3).toInt();
    printLine("OK");
    return;
  }

  // PitLogger extension: ATPLN1 = do not report the VIN (like some electric vehicles), ATPLN0 = report it
  if (a == "PLN0" || a == "PLN1") {
    hideVin = a[3] == '1';
    printLine("OK");
    return;
  }

  // PitLogger extension: list (ATPLV) or select (ATPLVn) the emulated vehicle
  if (a.startsWith("PLV")) {
    if (a.length() == 3) {
      printLine(String(vehicleSel == 0 ? "*" : " ") + "0 Simulated VW Golf (standard OBD-II only)");
      for (uint8_t i = 0; i < REC_VEHICLE_COUNT; i++) {
        printLine(String(vehicleSel == i + 1 ? "*" : " ") + (i + 1) + " " + REC_VEHICLES[i].name + ", " +
                  REC_VEHICLES[i].requestCount + " recorded requests");
      }
      return;
    }
    int v = a.substring(3).toInt();
    if (!isdigit(a[3]) || v < 0 || v > REC_VEHICLE_COUNT) { printLine("?"); return; }
    vehicleSel = v;
    prefs.putUChar("vehicle", vehicleSel);
    printLine("OK");
    return;
  }

  if (onOff(a, "E", cfg.echo) || onOff(a, "L", cfg.linefeed) ||
      onOff(a, "S", cfg.spaces) || onOff(a, "H", cfg.headers)) {
    printLine("OK");
    return;
  }

  if (a.startsWith("SP") || a.startsWith("TP")) {
    printLine(setProtocol(a.substring(2)) ? "OK" : "?");
    return;
  }

  // commands that are only acknowledged (timing, filters, headers, ...)
  static const char *ACK_PREFIXES[] = {
    "M", "AT", "CAF", "CFC", "AL", "NL", "ST", "SH", "CRA", "CF", "CM", "FC",
    "PC", "SW", "IB", "IIA", "KW", "BI", "V", "R", "CEA", "SS", "D0", "D1",
  };
  for (const char *p : ACK_PREFIXES) {
    if (a.startsWith(p)) { printLine("OK"); return; }
  }

  printLine("?");
}

// ---------------- Command processing ----------------
static void processLine(Port &port) {
  String cmd;
  if (port.lineOverflow) {
    printLine("?");
  } else {
    cmd = String(port.lineBuf).substring(0, port.lineLen);
    if (cmd.length() == 0) cmd = lastCmd;   // an empty line repeats the last command
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

// Both stacks run permanently; SerialBT.end() would shut down Bluedroid and with it BLE.
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
  adv->setScanResponse(true);   // the name is in the scan response
  BLEDevice::startAdvertising();
  btActive = true;              // both are visible after start-up
}

// Switches the visibility of both Bluetooth variants and drops existing connections.
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

// Open pin (internal pull-up) = Bluetooth on, connected to GND = off.
static bool btWanted() {
  return digitalRead(BT_SWITCH_PIN) == HIGH;
}

// Debounced: a new state only applies once it has been stable for BT_DEBOUNCE_MS.
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
  Serial.setRxBufferSize(4 * UPDATE_BLOCK);   // a whole update block arrives before it is read
  Serial.begin(ELM_BAUD);
  prefs.begin("emulator");
  vehicleSel = prefs.getUChar("vehicle", 1);
  if (vehicleSel > REC_VEHICLE_COUNT) vehicleSel = 0;
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
  canBegin();

  delay(200);
  io = &Serial;
  printLine(ELM_VERSION);
  eol();
  io->print('>');
}

void loop() {
  updateVehicle();
  checkBtSwitch();
  canLoop();
  pollPort(ports[0]);
  if (btActive) {
    pollPort(ports[1]);
    pollPort(ports[2]);
  }
}
