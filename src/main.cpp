#include <Arduino.h>
#include "driver/uart.h"

// ------- Pins & Baud ------
#define CRSF_UART_NUM UART_NUM_1
#define CRSF_TX_PIN   43   // ESP32 TX -> Module RX
#define CRSF_RX_PIN   -1   // Should be half-duplex...
#define CRSF_BAUD     1870000

#define HOST_BAUD     115200

// ------- Host Frame Types ------
#define HTYPE_CHANNELS   0x01
#define HTYPE_TEL_STATS  0x02
#define HTYPE_DEBUG      0x03
#define HTYPE_TEL_RAW    0x04

// ------- Framing ------
#define H0 0x55
#define H1 0xAA

// ------- CRSF constants ------
#define CRSF_SYNC                 0xC8
#define CRSF_TYPE_RC_CHANNELS     0x16
#define CRSF_TYPE_LINK_STATISTICS 0x14

// 16 channels, 11-bit each
static const int CHANNEL_COUNT = 16;

// -------- CRC8 (poly 0xD5) --------
uint8_t crc8_d5(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

void hostSend(uint8_t type, const uint8_t* payload, uint16_t plen) {
  uint8_t hdr[4] = {H0, H1, (uint8_t)(plen+1), (uint8_t)((plen+1)>>8)};
  Serial.write(hdr, 4);
  Serial.write(type);
  if (plen) Serial.write(payload, plen);
  uint8_t crc = crc8_d5(&type, 1);
  if (plen) crc = crc8_d5(payload, plen) ^ ((crc8_d5(&type,1)) ^ crc); // combine properly
  // Simpler and correct: compute over type+payload in one go
  uint8_t tmp[1 + (plen>0?plen:0)];
  tmp[0]=type; if (plen) memcpy(tmp+1,payload,plen);
  crc = crc8_d5(tmp, 1+plen);
  Serial.write(&crc, 1);
}

void hostSendDebug(const String& s) {
  hostSend(HTYPE_DEBUG, (const uint8_t*)s.c_str(), (uint16_t)s.length());
}

// ------ Pack 16x 11-bit channels into 22 bytes ------
void packCRSFChannels(const uint16_t ch[16], uint8_t out22[22]) {
  uint32_t bitbuf = 0; int bits = 0, idx = 0;
  memset(out22, 0, 22);
  for (int i = 0; i < 16; ++i) {
    bitbuf |= ((uint32_t)(ch[i] & 0x07FF)) << bits;
    bits += 11;
    while (bits >= 8) {
      out22[idx++] = bitbuf & 0xFF;
      bitbuf >>= 8; bits -= 8;
      if (idx >= 22) break;
    }
  }
  if (idx < 22) out22[idx++] = bitbuf & 0xFF;
}

void sendCRSFchannels(const uint16_t ch[16]) {
  uint8_t payload[22]; packCRSFChannels(ch, payload);
  uint8_t frame[1 + 1 + 1 + 22 + 1];
  int idx = 0;
  frame[idx++] = CRSF_SYNC;
  frame[idx++] = (uint8_t)(1 + 22 + 1); // type + payload + crc
  frame[idx++] = CRSF_TYPE_RC_CHANNELS;
  memcpy(&frame[idx], payload, 22); idx += 22;
  uint8_t crc = crc8_d5(&frame[2], 1 + 22); // over type+payload
  frame[idx++] = crc;
  uart_write_bytes(CRSF_UART_NUM, (const char*)frame, idx);
}

// ---- UART setup ----
void setupCRSFuart() {
  uart_config_t cfg = {
    .baud_rate = CRSF_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(CRSF_UART_NUM, &cfg);
  uart_set_pin(CRSF_UART_NUM, CRSF_TX_PIN, CRSF_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_line_inverse(CRSF_UART_NUM, UART_SIGNAL_TXD_INV);
  uart_driver_install(CRSF_UART_NUM, 1024, 1024, 0, NULL, 0);
}

// ----- CRSF telemetry parsing (best-effort for 0x14) -----
bool parseCRSFLinkStats(const uint8_t* payload, uint8_t plen) {
  // Typical ordering (may vary):
  // [0]=1RSS (dBm, int8), [1]=2RSS (dBm, int8), [2]=LQ (u8, %),
  // [3]=RSNR (dB, int8),  [4]=RFMD (u8),        [5]=TPWR (u8, dBm approx)
  // [6]=TRSS (dBm, int8), [7]=TLQ (u8, %),      [8]=TSNR (dB, int8), [9]=FLAGS(u8)
  if (plen < 6) return false;
  int8_t  rssi1 = (int8_t)payload[0];
  int8_t  rssi2 = (plen>1) ? (int8_t)payload[1] : rssi1;
  uint8_t lq    = (plen>2) ? payload[2] : 0;
  int8_t  rsnr  = (plen>3) ? (int8_t)payload[3] : 0;
  uint8_t rfmd  = (plen>4) ? payload[4] : 0;
  uint8_t tpwr  = (plen>5) ? payload[5] : 0;
  int8_t  trss  = (plen>6) ? (int8_t)payload[6] : 0;
  uint8_t tlq   = (plen>7) ? payload[7] : 0;
  int8_t  tsnr  = (plen>8) ? (int8_t)payload[8] : 0;
  uint8_t flags = (plen>9) ? payload[9] : 0;

  uint8_t out[10];
  out[0]= (uint8_t)rssi1; out[1]=(uint8_t)rssi2; out[2]=lq; out[3]=(uint8_t)rsnr;
  out[4]= rfmd; out[5]= tpwr; out[6]=(uint8_t)trss; out[7]=tlq; out[8]=(uint8_t)tsnr; out[9]=flags;
  hostSend(HTYPE_TEL_STATS, out, sizeof(out));
  return true;
}

void sendRawTel(const uint8_t* buf, int len) {
  hostSend(HTYPE_TEL_RAW, buf, (uint16_t)len);
}

// ----- Host frame parser -----
enum HostState { S0, S1, S2_LEN0, S3_LEN1, S4_BODY, S5_CRC };
static HostState hstate = S0;
static uint16_t hlen = 0;
static uint8_t htype = 0;
static uint8_t hcrc = 0;
static uint8_t hbuf[128];
static uint16_t hpos = 0;

uint32_t lastHostMs = 0, lastSweepMs = 0;

void handleHostFrame(uint8_t type, const uint8_t* p, uint16_t n) {
  if (type == HTYPE_CHANNELS && n == 32) {
    uint16_t ch[16];
    for (int i=0;i<16;i++){
      uint16_t v = (uint16_t)p[2*i] | ((uint16_t)p[2*i+1]<<8);
      if (v>2047) v=2047; ch[i]=v;
    }
    sendCRSFchannels(ch);
    lastHostMs = millis();
  }
  // other types reserved for future host->device commands
}

void pumpHostParser() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch(hstate){
      case S0: hstate=(b==H0)?S1:S0; break;
      case S1: hstate=(b==H1)?S2_LEN0:S0; break;
      case S2_LEN0: hlen=b; hstate=S3_LEN1; break;
      case S3_LEN1: hlen|=((uint16_t)b<<8); hpos=0; hstate=S4_BODY; break;
      case S4_BODY:
        if (hpos==0){ htype=b; }
        else { if (hpos-1<sizeof(hbuf)) hbuf[hpos-1]=b; }
        hpos++;
        if (hpos>=hlen){ hstate=S5_CRC; }
        break;
      case S5_CRC: {
        uint8_t tmp[1+sizeof(hbuf)];
        tmp[0]=htype; if (hlen>1) memcpy(tmp+1,hbuf,hlen-1);
        uint8_t crc = crc8_d5(tmp, hlen);
        if (crc==b) handleHostFrame(htype, hbuf, (uint16_t)(hlen-1));
        hstate=S0; break;
      }
    }
  }
}

// ----- CRSF UART RX -> parse frames -----
void pumpCRSF() {
  uint8_t buf[256];
  int r = uart_read_bytes(CRSF_UART_NUM, buf, sizeof(buf), 0);
  if (r<=0) return;

  // Basic scanner for one or more CRSF frames inside buf
  int i=0;
  while (i+3<=r) {
    if (buf[i]!=CRSF_SYNC) { i++; continue; }
    if (i+2>r) break;
    uint8_t len = buf[i+1];
    int needed = 2 + len; // includes type..payload..crc
    if (i+needed>r) break;
    uint8_t type = buf[i+2];
    const uint8_t* payload = &buf[i+3];
    uint8_t paylen = len-1-1; // remove type & crc
    uint8_t crc = buf[i+2+1+paylen]; // after type+payload
    // verify
    uint8_t tmp[1+255];
    tmp[0]=type; if (paylen) memcpy(tmp+1,payload,paylen);
    if (crc == crc8_d5(tmp, 1+paylen)) {
      if (type == CRSF_TYPE_LINK_STATISTICS) {
        parseCRSFLinkStats(payload, paylen);
      } else {
        // forward raw
        sendRawTel(&buf[i], needed);
      }
    }
    i += needed;
  }
}

void setup() {
  Serial.begin(HOST_BAUD);
  while(!Serial) { delay(10); }
  setupCRSFuart();
  hostSendDebug("ESP32 CRSF bridge ready");
}

void loop() {
  pumpHostParser();
  pumpCRSF();

  uint32_t now = millis();
  if (now - lastHostMs > 1000 && now - lastSweepMs > 50) {
    uint16_t ch[16];
    for (int i = 0; i < 16; i++) ch[i] = 1500; // default to 1500 on all
    ch[4] = 1000; // CH5 (1-based) at 1000
    sendCRSFchannels(ch);
    lastSweepMs = now;
  }
}
