#include <Arduino.h>

// ======================================================
// НАСТРОЙКИ UART (АМУР / AceUno / AceNano)
// ======================================================
// Подключение LD06:
//   TX LD06  -> RX1 (D7)  контроллера
//   RX LD06  -> не используется (лидар только передаёт)
//   GND      -> GND
//   VCC LD06 -> 5V (мотор) + 3.3V логика, смотри даташит модуля
//
// ======================================================

static const uint32_t LIDAR_BAUD = 230400;
static const uint32_t USB_BAUD   = 230400;

#define LidarSerial Serial1

// true  -> пересылать в USB Serial валидные пакеты LD06 "как есть"
#define USB_SERIAL_PASSTHROUGH_RAW true

// true  -> печатать текстовую отладку (сломает Python raw-режим!)
#define DEBUG_TEXT_OUTPUT false

// ======================================================
// КОНСТАНТЫ LD06
// ======================================================

static const uint8_t  LD06_HEADER = 0x54;
static const uint8_t  VERLEN_POINTS_MASK = 0x1F;
static const uint16_t FULL_CIRCLE_CDEG = 36000;

static const size_t STREAM_BUF_SIZE = 256;

uint8_t streamBuf[STREAM_BUF_SIZE];
size_t streamLen = 0;

// ======================================================
// СТРУКТУРЫ
// ======================================================

struct Ld06Point {
  uint16_t angle_cdeg;
  uint16_t dist_mm;
  uint8_t  intensity;
};

struct Ld06Frame {
  uint8_t  ver;
  uint8_t  n_points;
  uint16_t speed_dps;
  uint16_t start_cdeg;
  uint16_t end_cdeg;
  uint16_t timestamp_ms;
  Ld06Point points[12];
};

// ======================================================
// ВСПОМОГАТЕЛЬНЫЕ
// ======================================================

uint16_t le16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

size_t expectedPacketLen(uint8_t n_points) {
  return 11 + 3 * n_points;
}

uint8_t crc8_ldrobot(const uint8_t *data, size_t len,
                     uint8_t poly = 0x4D, uint8_t init = 0x00) {
  uint8_t crc = init;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int k = 0; k < 8; k++) {
      if (crc & 0x80) {
        crc = (uint8_t)(((crc << 1) ^ poly) & 0xFF);
      } else {
        crc = (uint8_t)((crc << 1) & 0xFF);
      }
    }
  }
  return crc;
}

// ======================================================
// ОТЛАДКА
// ======================================================

void printHex2(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void debugPrintFrame(const uint8_t *packet, size_t pktLen, const Ld06Frame &fr) {
#if DEBUG_TEXT_OUTPUT
  Serial.println();
  Serial.println("=========== LD06 FRAME ===========");

  Serial.print("RAW: ");
  for (size_t i = 0; i < pktLen; i++) {
    printHex2(packet[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print("points = "); Serial.println(fr.n_points);
  Serial.print("speed  = "); Serial.println(fr.speed_dps);
  Serial.print("start  = "); Serial.println(fr.start_cdeg / 100.0f);
  Serial.print("end    = "); Serial.println(fr.end_cdeg / 100.0f);
#endif
}

// ======================================================
// ПАРСИНГ ПАКЕТА
// ======================================================

bool parsePacket(const uint8_t *packet, size_t len, Ld06Frame &outFrame) {
  if (len < 11) return false;
  if (packet[0] != LD06_HEADER) return false;

  uint8_t ver_len = packet[1];
  uint8_t ver = (ver_len >> 5) & 0x07;
  uint8_t n   = ver_len & VERLEN_POINTS_MASK;

  if (n == 0) return false;
  if (len != expectedPacketLen(n)) return false;
  if (n > 12) return false;

  uint8_t crc_rx   = packet[len - 1];
  uint8_t crc_calc = crc8_ldrobot(packet, len - 1);
  if (crc_rx != crc_calc) return false;

  outFrame.ver          = ver;
  outFrame.n_points     = n;
  outFrame.speed_dps    = le16(&packet[2]);
  outFrame.start_cdeg   = le16(&packet[4]);
  outFrame.end_cdeg     = le16(&packet[len - 5]);
  outFrame.timestamp_ms = le16(&packet[len - 3]);

  return true;
}

// ======================================================
// ПОТОКОВЫЙ ПАРСЕР
// ======================================================

void processIncomingByte(uint8_t b) {
  if (streamLen < STREAM_BUF_SIZE) {
    streamBuf[streamLen++] = b;
  } else {
    memmove(streamBuf, streamBuf + 1, STREAM_BUF_SIZE - 1);
    streamBuf[STREAM_BUF_SIZE - 1] = b;
    streamLen = STREAM_BUF_SIZE;
  }

  while (true) {
    int idx = -1;
    for (size_t i = 0; i < streamLen; i++) {
      if (streamBuf[i] == LD06_HEADER) {
        idx = (int)i;
        break;
      }
    }

    if (idx < 0) {
      streamLen = 0;
      return;
    }

    if (idx > 0) {
      memmove(streamBuf, streamBuf + idx, streamLen - idx);
      streamLen -= idx;
    }

    if (streamLen < 2) return;

    uint8_t n = streamBuf[1] & VERLEN_POINTS_MASK;
    if (n == 0) {
      memmove(streamBuf, streamBuf + 1, streamLen - 1);
      streamLen--;
      continue;
    }

    size_t pktLen = expectedPacketLen(n);
    if (streamLen < pktLen) return;

    Ld06Frame frame;
    bool ok = parsePacket(streamBuf, pktLen, frame);

    if (!ok) {
      memmove(streamBuf, streamBuf + 1, streamLen - 1);
      streamLen--;
      continue;
    }

    // Отправка сырого пакета в Python
    if (USB_SERIAL_PASSTHROUGH_RAW) {
      Serial.write(streamBuf, pktLen);
    }

    debugPrintFrame(streamBuf, pktLen, frame);

    memmove(streamBuf, streamBuf + pktLen, streamLen - pktLen);
    streamLen -= pktLen;
  }
}

// ======================================================
// SETUP / LOOP
// ======================================================

void setup() {
  Serial.begin(USB_BAUD);
  delay(500);

  // На АМУР пины Serial1 фиксированы аппаратно (RX1=D7, TX1=D8)
  LidarSerial.begin(LIDAR_BAUD);

#if DEBUG_TEXT_OUTPUT
  Serial.println("LD06 passthrough started");
#endif
}

void loop() {
  while (LidarSerial.available()) {
    uint8_t b = (uint8_t)LidarSerial.read();
    processIncomingByte(b);
  }
}