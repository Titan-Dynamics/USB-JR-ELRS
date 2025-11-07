#include <Arduino.h>

/*
  Seeed XIAO ESP32-S3 → ELRS TX module (JR bay data pin)
  TX-only CRSF frame generator
  Baud: 400000, inverted line, 16ch sweep 1000–2000 µs
*/

//////////////////// User-tweakable parameters ////////////////////
static constexpr int   JR_TX_PIN      = 43;       // Good UART TX pin on XIAO ESP32-S3
static constexpr bool  INVERT_LINE    = true;     // JR bay CRSF is inverted logic
static constexpr uint32_t CRSF_BAUD   = 400000;   // 400 kbaud per spec
static constexpr uint32_t FRAME_HZ    = 250;      // 250 Hz channel output
static constexpr bool  USE_EDGETX_SYNC= false;     // Use 0xEE like EdgeTX
///////////////////////////////////////////////////////////////////

static constexpr uint8_t SYNC_BYTE = USE_EDGETX_SYNC ? 0xEE : 0xC8;
static constexpr uint8_t TYPE_RC_CHANNELS_PACKED = 0x16;
static constexpr size_t  PAYLOAD_LEN = 22;        // 16×11-bit channels
static constexpr uint8_t LEN_BYTE    = PAYLOAD_LEN + 2; // type+payload+crc
static constexpr uint32_t PERIOD_US  = 1000000UL / FRAME_HZ;

// Convert µs → CRSF channel value (172–1811)
static inline uint16_t usToCrsf(int us)
{
  long crsf = 992 + ((8L * (us - 1500)) / 5);
  if (crsf < 172)  crsf = 172;
  if (crsf > 1811) crsf = 1811;
  return (uint16_t)crsf & 0x7FF;
}

// CRC8 with poly 0xD5
static uint8_t crc8_d5(const uint8_t* data, size_t len)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (crc << 1);
    }
  }
  return crc;
}

// Pack 16×11-bit channel values (22 bytes)
static void packChannels(const uint16_t ch[16], uint8_t out[PAYLOAD_LEN])
{
  memset(out, 0, PAYLOAD_LEN);
  uint32_t bitpos = 0;
  for (int i = 0; i < 16; ++i) {
    uint32_t v = ch[i] & 0x7FF;
    for (int b = 0; b < 11; ++b) {
      if (v & (1u << b))
        out[bitpos >> 3] |= (1u << (bitpos & 7));
      ++bitpos;
    }
  }
}

// Build and send CRSF RC_CHANNELS frame
static void sendCrsfChannels(const uint16_t ch[16])
{
  uint8_t payload[PAYLOAD_LEN];
  packChannels(ch, payload);

  uint8_t frame[1 + 1 + 1 + PAYLOAD_LEN + 1];
  size_t idx = 0;
  frame[idx++] = SYNC_BYTE;
  frame[idx++] = LEN_BYTE;
  frame[idx++] = TYPE_RC_CHANNELS_PACKED;
  memcpy(&frame[idx], payload, PAYLOAD_LEN);
  idx += PAYLOAD_LEN;
  frame[idx++] = crc8_d5(&frame[2], 1 + PAYLOAD_LEN);

  Serial1.write(frame, sizeof(frame));
  Serial1.flush();
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("\nCRSF TX demo (XIAO ESP32-S3)");

  // UART1 TX-only, inverted
  Serial1.begin(CRSF_BAUD, SERIAL_8N1, -1, JR_TX_PIN, INVERT_LINE);

  // open-drain for half-duplex compatibility
  pinMode(JR_TX_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(JR_TX_PIN, HIGH);

  delay(100);
}

void loop()
{
  static uint32_t last = 0;
  const uint32_t now = micros();
  if ((uint32_t)(now - last) >= PERIOD_US) {
    last += PERIOD_US;

    // Simple sweep 1000–2000 µs
    static int us = 1000;
    static int dir = 1;
    us += dir * 5;
    if (us >= 2000) { us = 2000; dir = -1; }
    if (us <= 1000) { us = 1000; dir = 1; }

    uint16_t ch[16];
    const uint16_t v = usToCrsf(us);
    for (int i = 0; i < 16; ++i) ch[i] = v;

    sendCrsfChannels(ch);
  }
}
