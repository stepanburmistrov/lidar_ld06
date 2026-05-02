#include <Arduino.h>

/*
  ==============================================================
  LD06 -> ESP32 -> USB Serial (validated packet passthrough)
  --------------------------------------------------------------
  Назначение:
    - читать поток байт от лидара LD06 по UART;
    - искать в потоке границы пакетов;
    - проверять длину и CRC каждого пакета;
    - пересылать в USB Serial ТОЛЬКО валидные пакеты;
    - при необходимости печатать человекочитаемую отладку.

  Важная особенность этой версии:
    - скетч умеет ПОЛНОСТЬЮ декодировать пакет, включая точки;
    - но основной режим работы — именно "валидатор + passthrough".

  Это удобно, когда:
    - ESP32 используется как UART-мост/фильтр;
    - более тяжёлая обработка выполняется на ПК/Raspberry Pi;
    - Python/ROS/OpenCV должны получать только корректные пакеты.
  ==============================================================
*/

// ==============================================================
// НАСТРОЙКИ UART
// ==============================================================

static const int RX_PIN = 16;       // TX LD06 -> RX2 ESP32
static const int TX_PIN = 17;       // для LD06 обычно не используется

static const uint32_t LIDAR_BAUD = 230400;
static const uint32_t USB_BAUD   = 230400;

#define LidarSerial Serial2

// true  -> пересылать в USB Serial только корректные пакеты LD06
// false -> не пересылать пакеты, использовать только локальную обработку/отладку
#define USB_SERIAL_PASSTHROUGH_RAW true

// true  -> печатать текстовую отладку в Serial
// ВАЖНО: если это включить, Python-программа, ожидающая "сырой" бинарный поток,
//        больше не сможет напрямую читать этот же порт как поток пакетов LD06.
#define DEBUG_TEXT_OUTPUT false

// true  -> печатать подробную информацию по точкам пакета
// Обычно выключено, иначе вывод будет очень объёмным.
#define DEBUG_PRINT_POINTS false

// ==============================================================
// КОНСТАНТЫ ПРОТОКОЛА LD06
// ==============================================================

static const uint8_t  LD06_HEADER = 0x54;
static const uint8_t  VERLEN_POINTS_MASK = 0x1F;  // младшие 5 бит: число точек
static const uint16_t FULL_CIRCLE_CDEG = 36000;  // 360.00° в centi-deg

// Максимальный размер поточного буфера.
// 256 байт более чем достаточно для пакетов LD06, но оставляет запас,
// если в потоке есть мусор до первого корректного заголовка.
static const size_t STREAM_BUF_SIZE = 256;

// ==============================================================
// БУФЕР ПОТОКА И СЧЁТЧИКИ СТАТИСТИКИ
// ==============================================================

uint8_t streamBuf[STREAM_BUF_SIZE];
size_t streamLen = 0;

uint32_t g_bytesIn           = 0;
uint32_t g_framesOk          = 0;
uint32_t g_crcErrors         = 0;
uint32_t g_headerMissErrors  = 0;
uint32_t g_lengthErrors      = 0;
uint32_t g_overflowEvents    = 0;
uint32_t g_zeroPointErrors   = 0;
uint32_t g_tooManyPointErr   = 0;

unsigned long g_lastStatsMs = 0;

// ==============================================================
// СТРУКТУРЫ ДАННЫХ
// ==============================================================

struct Ld06Point {
  uint16_t angle_cdeg;  // угол точки в centi-deg
  uint16_t dist_mm;     // дистанция в мм
  uint8_t intensity;    // интенсивность отражения
};

struct Ld06Frame {
  uint8_t ver;          // версия/служебные биты из ver_len
  uint8_t n_points;     // число точек в пакете
  uint16_t speed_dps;   // скорость вращения, deg/s
  uint16_t start_cdeg;  // начальный угол пакета
  uint16_t end_cdeg;    // конечный угол пакета
  uint16_t timestamp_ms;
  Ld06Point points[12]; // LD06 обычно передаёт до 12 точек в пакете
};

// ==============================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ==============================================================

// Чтение little-endian uint16_t из массива байт.
uint16_t le16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// Длина полного пакета LD06 для n точек.
// Формат: 11 + 3*n байт
//   1 байт  header
//   1 байт  ver_len
//   2 байта speed
//   2 байта start angle
//   3*n     точки
//   2 байта end angle
//   2 байта timestamp
//   1 байт  CRC
size_t expectedPacketLen(uint8_t n_points) {
  return 11 + 3 * (size_t)n_points;
}

// CRC-8 для LDROBOT/LD06.
// Полином инициализации взяты из рабочей практики по этим лидарам:
//   poly = 0x4D
//   init = 0x00
uint8_t crc8_ldrobot(const uint8_t *data, size_t len, uint8_t poly = 0x4D, uint8_t init = 0x00) {
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

// Безопасная разница углов на окружности в centi-deg.
// Нужна, чтобы корректно интерполировать точки даже при переходе через 359.99° -> 0.00°.
uint16_t circularDeltaCdeg(uint16_t from_cdeg, uint16_t to_cdeg) {
  return (uint16_t)((to_cdeg + FULL_CIRCLE_CDEG - from_cdeg) % FULL_CIRCLE_CDEG);
}

// ==============================================================
// ОТЛАДОЧНЫЙ ВЫВОД
// ==============================================================

void printHex2(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void printStatsIfNeeded() {
#if DEBUG_TEXT_OUTPUT
  unsigned long now = millis();
  if (now - g_lastStatsMs >= 2000UL) {
    Serial.println();
    Serial.println(F("========== LD06 STATS =========="));
    Serial.print(F("bytes_in          = ")); Serial.println(g_bytesIn);
    Serial.print(F("frames_ok         = ")); Serial.println(g_framesOk);
    Serial.print(F("crc_errors        = ")); Serial.println(g_crcErrors);
    Serial.print(F("header_errors     = ")); Serial.println(g_headerMissErrors);
    Serial.print(F("length_errors     = ")); Serial.println(g_lengthErrors);
    Serial.print(F("zero_point_errors = ")); Serial.println(g_zeroPointErrors);
    Serial.print(F("too_many_points   = ")); Serial.println(g_tooManyPointErr);
    Serial.print(F("overflow_events   = ")); Serial.println(g_overflowEvents);
    Serial.print(F("stream_len        = ")); Serial.println(streamLen);
    g_lastStatsMs = now;
  }
#endif
}

void debugPrintFrame(const uint8_t *packet, size_t pktLen, const Ld06Frame &fr) {
#if DEBUG_TEXT_OUTPUT
  Serial.println();
  Serial.println(F("=========== LD06 FRAME ==========="));

  Serial.print(F("RAW: "));
  for (size_t i = 0; i < pktLen; i++) {
    printHex2(packet[i]);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print(F("ver    = ")); Serial.println(fr.ver);
  Serial.print(F("points = ")); Serial.println(fr.n_points);
  Serial.print(F("speed  = ")); Serial.print(fr.speed_dps); Serial.println(F(" deg/s"));
  Serial.print(F("start  = ")); Serial.print(fr.start_cdeg / 100.0f); Serial.println(F(" deg"));
  Serial.print(F("end    = ")); Serial.print(fr.end_cdeg / 100.0f); Serial.println(F(" deg"));
  Serial.print(F("time   = ")); Serial.print(fr.timestamp_ms); Serial.println(F(" ms"));

  #if DEBUG_PRINT_POINTS
  for (uint8_t i = 0; i < fr.n_points; i++) {
    Serial.print(F("  pt[")); Serial.print(i); Serial.print(F("]: angle="));
    Serial.print(fr.points[i].angle_cdeg / 100.0f);
    Serial.print(F(" deg, dist="));
    Serial.print(fr.points[i].dist_mm);
    Serial.print(F(" mm, intensity="));
    Serial.println(fr.points[i].intensity);
  }
  #endif
#endif
}

// ==============================================================
// ПАРСИНГ ОДНОГО ПАКЕТА
// ==============================================================

bool parsePacket(const uint8_t *packet, size_t len, Ld06Frame &outFrame) {
  // Минимальная длина пакета LD06 при 0 точках всё равно 11,
  // но 0 точек для нас считаем ошибкой.
  if (len < 11) {
    g_lengthErrors++;
    return false;
  }

  if (packet[0] != LD06_HEADER) {
    g_headerMissErrors++;
    return false;
  }

  const uint8_t ver_len = packet[1];
  const uint8_t ver = (ver_len >> 5) & 0x07;
  const uint8_t n   = ver_len & VERLEN_POINTS_MASK;

  if (n == 0) {
    g_zeroPointErrors++;
    return false;
  }

  if (n > 12) {
    g_tooManyPointErr++;
    return false;
  }

  if (len != expectedPacketLen(n)) {
    g_lengthErrors++;
    return false;
  }

  const uint8_t crc_rx   = packet[len - 1];
  const uint8_t crc_calc = crc8_ldrobot(packet, len - 1);

  if (crc_rx != crc_calc) {
    g_crcErrors++;
    return false;
  }

  outFrame.ver          = ver;
  outFrame.n_points     = n;
  outFrame.speed_dps    = le16(&packet[2]);
  outFrame.start_cdeg   = le16(&packet[4]);
  outFrame.end_cdeg     = le16(&packet[len - 5]);
  outFrame.timestamp_ms = le16(&packet[len - 3]);

  // Разбор точек.
  // LD06 передаёт только дистанцию и интенсивность для каждой точки.
  // Угол каждой точки обычно интерполируется между start_cdeg и end_cdeg.
  // Для n=1 шаг равен 0.
  float step_cdeg = 0.0f;
  if (n > 1) {
    const uint16_t delta = circularDeltaCdeg(outFrame.start_cdeg, outFrame.end_cdeg);
    step_cdeg = (float)delta / (float)(n - 1);
  }

  size_t offset = 6;
  for (uint8_t i = 0; i < n; i++) {
    const uint16_t dist_mm = le16(&packet[offset]);
    const uint8_t intensity = packet[offset + 2];
    offset += 3;

    uint16_t angle_cdeg = outFrame.start_cdeg;
    if (n > 1) {
      const int angle_i = (int)(outFrame.start_cdeg + step_cdeg * i + 0.5f);
      angle_cdeg = (uint16_t)(angle_i % FULL_CIRCLE_CDEG);
    }

    outFrame.points[i].angle_cdeg = angle_cdeg;
    outFrame.points[i].dist_mm = dist_mm;
    outFrame.points[i].intensity = intensity;
  }

  return true;
}

// ==============================================================
// ПОТОКОВЫЙ ПАРСЕР
// ==============================================================

void processIncomingByte(uint8_t b) {
  g_bytesIn++;

  // 1) Добавляем байт в хвост буфера.
  // Если буфер переполнен, делаем "скользящее окно":
  // выбрасываем самый старый байт и сохраняем новый.
  if (streamLen < STREAM_BUF_SIZE) {
    streamBuf[streamLen++] = b;
  } else {
    memmove(streamBuf, streamBuf + 1, STREAM_BUF_SIZE - 1);
    streamBuf[STREAM_BUF_SIZE - 1] = b;
    streamLen = STREAM_BUF_SIZE;
    g_overflowEvents++;
  }

  // 2) Пытаемся разобрать из накопленного буфера максимально возможное число пакетов.
  while (true) {
    // Ищем первый заголовок 0x54.
    int idx = -1;
    for (size_t i = 0; i < streamLen; i++) {
      if (streamBuf[i] == LD06_HEADER) {
        idx = (int)i;
        break;
      }
    }

    // Заголовок не найден: буфер полностью бесполезен, очищаем.
    if (idx < 0) {
      streamLen = 0;
      return;
    }

    // Если до заголовка был мусор — сдвигаем буфер влево.
    if (idx > 0) {
      memmove(streamBuf, streamBuf + idx, streamLen - idx);
      streamLen -= (size_t)idx;
    }

    // Нужно хотя бы 2 байта, чтобы узнать n_points из поля ver_len.
    if (streamLen < 2) return;

    const uint8_t n = streamBuf[1] & VERLEN_POINTS_MASK;

    if (n == 0) {
      // Псевдозаголовок или мусор: отбрасываем один байт и пробуем снова.
      memmove(streamBuf, streamBuf + 1, streamLen - 1);
      streamLen--;
      g_zeroPointErrors++;
      continue;
    }

    if (n > 12) {
      memmove(streamBuf, streamBuf + 1, streamLen - 1);
      streamLen--;
      g_tooManyPointErr++;
      continue;
    }

    const size_t pktLen = expectedPacketLen(n);

    // Пакет ещё не накопился целиком.
    if (streamLen < pktLen) return;

    Ld06Frame frame;
    const bool ok = parsePacket(streamBuf, pktLen, frame);

    if (!ok) {
      // Неудачный пакет: сдвигаем окно на 1 байт.
      // Это классическая стратегия ресинхронизации в бинарных протоколах.
      memmove(streamBuf, streamBuf + 1, streamLen - 1);
      streamLen--;
      continue;
    }

    g_framesOk++;

    // Если нужен режим прозрачной пересылки валидных пакетов в ПК.
    if (USB_SERIAL_PASSTHROUGH_RAW) {
      Serial.write(streamBuf, pktLen);
    }

    debugPrintFrame(streamBuf, pktLen, frame);

    // Удаляем разобранный пакет из начала буфера.
    memmove(streamBuf, streamBuf + pktLen, streamLen - pktLen);
    streamLen -= pktLen;
  }
}

// ==============================================================
// SETUP / LOOP
// ==============================================================

void setup() {
  Serial.begin(USB_BAUD);
  delay(500);

  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

#if DEBUG_TEXT_OUTPUT
  Serial.println(F("LD06 validated passthrough started"));
  Serial.print(F("USB baud   = ")); Serial.println(USB_BAUD);
  Serial.print(F("LIDAR baud = ")); Serial.println(LIDAR_BAUD);
#endif

  g_lastStatsMs = millis();
}

void loop() {
  while (LidarSerial.available()) {
    const uint8_t b = (uint8_t)LidarSerial.read();
    processIncomingByte(b);
  }

  printStatsIfNeeded();
}
