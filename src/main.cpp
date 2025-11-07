#include <Arduino.h>

// ===== User configuration =====
// JR bay signal pin (Arduino Nano ESP32: change to a convenient GPIO routed to the module pin)
 #ifndef JR_TX_PIN
 #define JR_TX_PIN -1
 #endif

// Optional RX pin if you want to read from the module (not used here)
 #ifndef JR_RX_PIN
 #define JR_RX_PIN -1
 #endif

// CRSF protocol settings
static const uint32_t CRSF_BAUD = 420000; // 420k 8N1
static const uint8_t CRSF_ADDRESS_MODULE = 0xC8; // destination address (TX module)
static const uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
static const uint8_t CRSF_RC_CHANNELS_PAYLOAD_LEN = 22; // 16 channels packed at 11 bits = 22 bytes
static const uint8_t CRSF_RC_FRAME_LENGTH = 24;          // length field: TYPE (1) + PAYLOAD (22) + CRC (1)

// Channel update / failsafe settings
static const uint32_t CHANNEL_TIMEOUT_MS = 500;  // if no host update within this time, hold last or set default
static const uint32_t CRSF_SEND_RATE_HZ = 150;   // send at ~150 Hz

// ===== Internal state =====
static uint16_t ch_us[16];            // channels in microseconds [1000..2000]
static uint32_t lastHostUpdateMs = 0; // millis of last valid host frame

// ===== Utilities =====
static inline uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Map microseconds [1000..2000] to CRSF 11-bit range [172..1811]
static inline uint16_t usToCrsfVal(uint16_t us) {
  us = clampU16(us, 1000, 2000);
  // Integer mapping: 1000 -> 172, 2000 -> 1811
  // value = 172 + (us - 1000) * (1811 - 172) / 1000
  return (uint16_t)(172 + ((uint32_t)(us - 1000) * (1811 - 172)) / 1000);
}

// CRC8 using polynomial 0xD5, initial 0x00, on TYPE + PAYLOAD
static uint8_t crc8_d5(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    uint8_t in = data[i] ^ crc;
    for (uint8_t b = 0; b < 8; b++) {
      if (in & 0x80) {
        in = (uint8_t)((in << 1) ^ 0xD5);
      } else {
        in <<= 1;
      }
    }
    crc = in;
  }
  return crc;
}

// Pack 16 channels of 11-bit values into 22 bytes, LSB-first packing
static void packCrsfChannels(const uint16_t chUs[16], uint8_t out[CRSF_RC_CHANNELS_PAYLOAD_LEN]) {
  uint16_t ch[16];
  for (int i = 0; i < 16; ++i) ch[i] = usToCrsfVal(chUs[i]) & 0x07FF;

  uint32_t bitPos = 0;
  memset(out, 0, CRSF_RC_CHANNELS_PAYLOAD_LEN);

  for (int i = 0; i < 16; ++i) {
    uint32_t val = ch[i];
    for (int b = 0; b < 11; ++b, ++bitPos) {
      uint32_t byteIndex = bitPos >> 3;      // div 8
      uint32_t bitIndex = bitPos & 0x7;      // mod 8
      if (val & (1u << b)) {
        out[byteIndex] |= (1u << bitIndex);
      }
    }
  }
}

static HardwareSerial &JR = Serial1;

// Build and send a CRSF RC channels frame to the JR bay UART
static void sendCrsfRcFrame(const uint16_t chUs[16]) {
  uint8_t payload[CRSF_RC_CHANNELS_PAYLOAD_LEN];
  packCrsfChannels(chUs, payload);

  uint8_t frame[2 /*addr+len*/ + 1 /*type*/ + CRSF_RC_CHANNELS_PAYLOAD_LEN + 1 /*crc*/];
  frame[0] = CRSF_ADDRESS_MODULE;
  frame[1] = CRSF_RC_FRAME_LENGTH; // includes type + payload + crc
  frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
  memcpy(&frame[3], payload, CRSF_RC_CHANNELS_PAYLOAD_LEN);
  uint8_t crc = crc8_d5(&frame[2], 1 + CRSF_RC_CHANNELS_PAYLOAD_LEN);
  frame[3 + CRSF_RC_CHANNELS_PAYLOAD_LEN] = crc;

  JR.write(frame, sizeof(frame));
}

// ===== Simple host protocol over USB CDC =====
// Two formats supported:
// 1) ASCII line: 16 integers separated by comma or space (1000..2000), newline-terminated
//    Example: "1500,1500,1500,1500,1000,2000, ...\n"
// 2) Binary: Magic 0xA5 0x5A, then 16 little-endian uint16 (1000..2000), then 1-byte checksum (sum of all payload bytes mod 256)

static bool parseAsciiLine(const String &line) {
  int idx = 0;
  int start = 0;
  int count = 0;
  uint16_t tmp[16];

  while (count < 16) {
    int sep = line.indexOf(',', start);
    int sep2 = line.indexOf(' ', start);
    if (sep == -1 || (sep2 != -1 && sep2 < sep)) sep = sep2;

    String token = (sep == -1) ? line.substring(start) : line.substring(start, sep);
    token.trim();
    if (token.length() == 0) break;
    long v = token.toInt();
    if (v == 0 && token != "0") return false; // not a number
    tmp[count++] = clampU16((uint16_t)v, 1000, 2000);
    if (sep == -1) break;
    start = sep + 1;
  }
  if (count != 16) return false;
  memcpy(ch_us, tmp, sizeof(ch_us));
  lastHostUpdateMs = millis();
  return true;
}

static bool parseBinary(Stream &s) {
  // We already saw first magic 0xA5 from caller
  if (!s.available()) return false;
  int b1 = s.read();
  if (b1 != 0x5A) return false;

  uint8_t payload[16 * 2];
  uint8_t checksum = 0;
  const size_t need_payload = sizeof(payload);
  uint32_t t0 = millis();
  size_t got = 0;
  while (got < need_payload && (millis() - t0) < 20) {
    if (s.available()) {
      int b = s.read();
      if (b < 0) continue;
      payload[got++] = (uint8_t)b;
    }
  }
  if (got != need_payload) return false;

  // Read checksum byte
  t0 = millis();
  while (!s.available() && (millis() - t0) < 5) {
    ;
  }
  if (!s.available()) return false;
  checksum = (uint8_t)s.read();

  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(payload); ++i) sum += payload[i];
  if (sum != checksum) return false;

  for (int i = 0; i < 16; ++i) {
    uint16_t v = payload[i * 2] | ((uint16_t)payload[i * 2 + 1] << 8);
    ch_us[i] = clampU16(v, 1000, 2000);
  }
  lastHostUpdateMs = millis();
  return true;
}

// ===== Arduino setup/loop =====
void setup() {
  // Default channels: 1500, except CH5 (aux1) 1000
  for (int i = 0; i < 16; ++i) ch_us[i] = 1500;
  ch_us[4] = 1000; // CH5 default low

  Serial.begin(115200); // USB CDC
  // Configure JR bay UART on default board pins (TX1/RX1)
  JR.begin(CRSF_BAUD);

  // Small banner
  delay(300);
  for (int i = 0; i < 3; ++i) {
    Serial.println("ELRS JR Bridge (ESP32-S3) starting...");
    Serial.print("JR TX pin: "); Serial.println(JR_TX_PIN);
    Serial.print("CRSF baud: "); Serial.println(CRSF_BAUD);
    Serial.println("Input: ASCII 16ch per line or binary 0xA5 0x5A + 16x u16 + sum");
    delay(200);
  }
}

void loop() {
  // Handle USB input
  while (Serial.available()) {
    int b = Serial.peek();
    if (b == 0xA5) {
      // consume and parse binary frame
      Serial.read();
      (void)parseBinary(Serial);
    } else {
      // read one line
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) {
        (void)parseAsciiLine(line);
      }
    }
  }

  static uint32_t lastSend = 0;
  const uint32_t now = millis();
  const uint32_t period = 1000 / CRSF_SEND_RATE_HZ;
  if (now - lastSend >= period) {
    // Failsafe/hold logic
    if (now - lastHostUpdateMs > CHANNEL_TIMEOUT_MS) {
      // Set channels to safe defaults (1500, CH5 low)
      for (int i = 0; i < 16; ++i) ch_us[i] = 1500;
      ch_us[4] = 1000;
    }
    sendCrsfRcFrame(ch_us);
    lastSend = now;
  }
}
