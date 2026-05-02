#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
======================================================================
LD06 live reader / scan assembler / XY viewer
----------------------------------------------------------------------
Назначение:
  - читать бинарный поток LD06-пакетов из COM-порта;
  - валидировать и декодировать пакеты;
  - собирать последовательность пакетов в скан 360°;
  - отображать точки в XY-плоскости через matplotlib;
  - при необходимости сохранять сканы в CSV.

Ожидаемый источник данных:
  - либо LD06 подключён напрямую к USB-UART;
  - либо ESP32 пересылает в USB Serial только валидные пакеты LD06.

Горячие клавиши в окне:
  q  - закрыть окно
  s  - сохранить текущий скриншот
======================================================================
"""

from __future__ import annotations

import csv
import logging
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from queue import Empty, Full, Queue
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation


# ============================================================
# НАСТРОЙКИ
# ============================================================

# --- COM-порт / скорость ---
PORT = "COM9"
BAUD = 230400
SERIAL_TIMEOUT = 0.05
SERIAL_READ_CHUNK = 4096

# --- Логирование / экспорт ---
LOG_FILE = "ld06_reader.log"
EXPORT_CSV = False
EXPORT_DIR = "out_scans"

# --- Фильтры по дистанции ---
MIN_RANGE_M = 0.02
MAX_RANGE_M = 12.0

# --- Фильтры по интенсивности ---
FILTER_BY_INTENSITY = False
MIN_INTENSITY = 0

# --- Геометрические настройки ---
# Поворот системы координат относительно данных лидара.
# Удобно, если вы хотите развернуть облако точек, не меняя сырых данных.
ANGLE_OFFSET_DEG = 0.0

# Зеркалирование осей.
# Часто полезно, если нужно подстроиться под конкретное размещение лидара на роботе.
INVERT_X = False
INVERT_Y = True   # True повторяет привычный экранный вариант, где "вперёд" визуально вверх

# --- Сборка сканов ---
# Если start_angle нового пакета стал меньше предыдущего, считаем, что закончился полный оборот.
WRAP_THRESHOLD_CDEG = 0

# --- Визуализация ---
PLOT_SCALE_M = 5.0
POINT_SIZE = 3.0
POINT_ALPHA = 0.90
ANIM_INTERVAL_MS = 100

# --- Цвета точек по интенсивности ---
USE_INTENSITY_COLORS = True
INTENSITY_MIN = 0
INTENSITY_MAX = 255
COLORMAP = "jet"  # turbo / plasma / viridis / inferno / jet

# --- Сохранение скриншотов ---
SNAPSHOT_DPI = 150


# ============================================================
# КОНСТАНТЫ LD06
# ============================================================

LD06_HEADER = 0x54
VERLEN_POINTS_MASK = 0x1F
FULL_CIRCLE_CDEG = 36000
MAX_POINTS_PER_PACKET = 12


# ============================================================
# СТРУКТУРЫ ДАННЫХ
# ============================================================

@dataclass(frozen=True)
class Ld06Point:
    """Одна измеренная точка после полной интерпретации."""

    angle_deg: float
    distance_m: float
    intensity: int
    x_m: float
    y_m: float


@dataclass(frozen=True)
class Ld06Frame:
    """Один валидный пакет LD06."""

    ver: int
    n_points: int
    speed_dps: int
    start_cdeg: int
    end_cdeg: int
    timestamp_ms: int
    points: Tuple[Ld06Point, ...]


@dataclass(frozen=True)
class ScanBundle:
    """Один собранный скан, примерно соответствующий полному обороту лидара."""

    scan_ts_unix: float
    speed_dps: int
    points: Tuple[Ld06Point, ...]


# ============================================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ============================================================


def crc8_ldrobot(data: bytes, poly: int = 0x4D, init: int = 0x00) -> int:
    """Вычисление CRC-8 для протокола LD06."""
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc



def expected_packet_len(n_points: int) -> int:
    """Вернуть длину пакета LD06 по числу точек."""
    return 11 + 3 * n_points



def circular_delta_cdeg(start_cdeg: int, end_cdeg: int) -> int:
    """Положительная разница углов на окружности в centi-deg."""
    return (end_cdeg - start_cdeg) % FULL_CIRCLE_CDEG



def normalize_angle_deg(angle_deg: float) -> float:
    """Нормализация угла в диапазон [0, 360)."""
    return angle_deg % 360.0



def polar_to_xy(distance_m: float, angle_deg: float) -> Tuple[float, float]:
    """
    Преобразование полярных координат в декартовы.

    По умолчанию используется инженерная схема:
      x = r * cos(theta)
      y = r * sin(theta)

    Затем при необходимости применяются инверсия осей.
    """
    theta = math.radians(angle_deg)
    x = distance_m * math.cos(theta)
    y = distance_m * math.sin(theta)

    if INVERT_X:
        x = -x
    if INVERT_Y:
        y = -y

    return x, y


# ============================================================
# ПАРСИНГ ПАКЕТА LD06
# ============================================================


def parse_packet(packet: bytes) -> Optional[Ld06Frame]:
    """
    Разобрать один полный пакет LD06.

    Возвращает:
      - Ld06Frame, если пакет корректен;
      - None, если пакет повреждён или имеет неверную структуру.
    """
    if len(packet) < 11:
        return None

    if packet[0] != LD06_HEADER:
        return None

    ver_len = packet[1]
    ver = (ver_len >> 5) & 0x07
    n_points = ver_len & VERLEN_POINTS_MASK

    if n_points <= 0 or n_points > MAX_POINTS_PER_PACKET:
        return None

    if len(packet) != expected_packet_len(n_points):
        return None

    crc_rx = packet[-1]
    crc_calc = crc8_ldrobot(packet[:-1])
    if crc_rx != crc_calc:
        return None

    speed_dps = int.from_bytes(packet[2:4], byteorder="little", signed=False)
    start_cdeg = int.from_bytes(packet[4:6], byteorder="little", signed=False)
    end_cdeg = int.from_bytes(packet[-5:-3], byteorder="little", signed=False)
    timestamp_ms = int.from_bytes(packet[-3:-1], byteorder="little", signed=False)

    # Интерполяция углов точек внутри пакета.
    if n_points == 1:
        step_cdeg = 0.0
    else:
        delta_cdeg = circular_delta_cdeg(start_cdeg, end_cdeg)
        step_cdeg = delta_cdeg / float(n_points - 1)

    points: List[Ld06Point] = []
    offset = 6

    for i in range(n_points):
        dist_mm = int.from_bytes(packet[offset:offset + 2], byteorder="little", signed=False)
        intensity = int(packet[offset + 2])
        offset += 3

        angle_cdeg = int((start_cdeg + step_cdeg * i) % FULL_CIRCLE_CDEG)
        angle_deg = normalize_angle_deg(angle_cdeg / 100.0 + ANGLE_OFFSET_DEG)
        distance_m = dist_mm / 1000.0

        # Фильтр по дальности
        if distance_m < MIN_RANGE_M or distance_m > MAX_RANGE_M:
            continue

        # Фильтр по интенсивности
        if FILTER_BY_INTENSITY and intensity < MIN_INTENSITY:
            continue

        x_m, y_m = polar_to_xy(distance_m, angle_deg)

        points.append(
            Ld06Point(
                angle_deg=angle_deg,
                distance_m=distance_m,
                intensity=intensity,
                x_m=x_m,
                y_m=y_m,
            )
        )

    return Ld06Frame(
        ver=ver,
        n_points=n_points,
        speed_dps=speed_dps,
        start_cdeg=start_cdeg,
        end_cdeg=end_cdeg,
        timestamp_ms=timestamp_ms,
        points=tuple(points),
    )


# ============================================================
# ПОТОКОВЫЙ ПАРСЕР
# ============================================================


class Ld06StreamParser:
    """
    Инкрементальный парсер байтового потока.

    Идея работы:
      1. В буфер добавляются новые байты;
      2. Ищется первый возможный заголовок 0x54;
      3. По полю ver_len определяется ожидаемая длина пакета;
      4. Пакет проверяется через parse_packet;
      5. При успехе пакет выдаётся наружу, при провале выполняется ресинхронизация.
    """

    def __init__(self, logger: logging.Logger):
        self.buf = bytearray()
        self.log = logger

        # Статистика
        self.bytes_in = 0
        self.frames_ok = 0
        self.crc_errors = 0
        self.invalid_header = 0
        self.invalid_n_points = 0
        self.resync_shifts = 0

    def feed(self, data: bytes) -> List[Ld06Frame]:
        """Подать новую порцию байт и вернуть все найденные валидные пакеты."""
        self.buf.extend(data)
        self.bytes_in += len(data)

        frames: List[Ld06Frame] = []

        while True:
            idx = self.buf.find(bytes([LD06_HEADER]))

            if idx < 0:
                # Заголовка нет: нет смысла держать весь мусор.
                # Оставим только последний байт на случай, если он начало будущего шаблона.
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                break

            if idx > 0:
                del self.buf[:idx]
                self.resync_shifts += idx

            if len(self.buf) < 2:
                break

            ver_len = self.buf[1]
            n_points = ver_len & VERLEN_POINTS_MASK

            if n_points <= 0 or n_points > MAX_POINTS_PER_PACKET:
                del self.buf[:1]
                self.invalid_n_points += 1
                self.resync_shifts += 1
                continue

            pkt_len = expected_packet_len(n_points)
            if len(self.buf) < pkt_len:
                break

            packet = bytes(self.buf[:pkt_len])
            frame = parse_packet(packet)

            if frame is None:
                # Чаще всего сюда попадают ошибки CRC или ложные совпадения по заголовку.
                # Сдвигаемся на 1 байт и пробуем найти следующую синхронизацию.
                self.crc_errors += 1
                del self.buf[:1]
                self.resync_shifts += 1
                continue

            frames.append(frame)
            self.frames_ok += 1
            del self.buf[:pkt_len]

        return frames


# ============================================================
# СБОРЩИК ПОЛНЫХ СКАНОВ
# ============================================================


class ScanAssembler:
    """
    Собирает последовательность пакетов в полный оборот лидара.

    Логика завершения скана простая и практичная:
      - пока start_angle возрастает, считаем, что это текущий оборот;
      - когда очередной start_angle становится меньше предыдущего,
        считаем, что лидар начал новый оборот;
      - накопленные до этого точки выдаём как готовый ScanBundle.

    Это не единственно возможная стратегия, но для live-визуализации работает хорошо.
    """

    def __init__(self):
        self._accum: List[Ld06Point] = []
        self._prev_start_cdeg: Optional[int] = None
        self._last_speed_dps: int = 0

    def push_frame(self, frame: Ld06Frame) -> Optional[ScanBundle]:
        bundle: Optional[ScanBundle] = None

        # Признак перехода к новому обороту.
        if (
            self._prev_start_cdeg is not None
            and frame.start_cdeg + WRAP_THRESHOLD_CDEG < self._prev_start_cdeg
        ):
            if self._accum:
                bundle = ScanBundle(
                    scan_ts_unix=time.time(),
                    speed_dps=self._last_speed_dps,
                    points=tuple(self._accum),
                )
                self._accum = []

        self._accum.extend(frame.points)
        self._prev_start_cdeg = frame.start_cdeg
        self._last_speed_dps = frame.speed_dps
        return bundle


# ============================================================
# ЭКСПОРТ CSV
# ============================================================


def export_scan_csv(path: str, scan: ScanBundle) -> None:
    """Сохранить один скан в CSV."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, delimiter=";")
        writer.writerow([
            "scan_ts_unix",
            "speed_dps",
            "angle_deg",
            "distance_m",
            "intensity",
            "x_m",
            "y_m",
        ])
        for p in scan.points:
            writer.writerow([
                scan.scan_ts_unix,
                scan.speed_dps,
                p.angle_deg,
                p.distance_m,
                p.intensity,
                p.x_m,
                p.y_m,
            ])


# ============================================================
# ПОТОК ЧТЕНИЯ ИЗ SERIAL
# ============================================================


def reader_loop(q: Queue, stop_evt: threading.Event, logger: logging.Logger) -> None:
    """
    Фоновый поток:
      - читает данные из serial;
      - разбирает пакеты;
      - собирает сканы;
      - кладёт в очередь только самый свежий скан.
    """
    logger.info("Opening serial port: port=%s baud=%d", PORT, BAUD)

    with serial.Serial(
        port=PORT,
        baudrate=BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=SERIAL_TIMEOUT,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    ) as ser:
        parser = Ld06StreamParser(logger)
        assembler = ScanAssembler()

        last_stat = time.time()

        while not stop_evt.is_set():
            chunk = ser.read(SERIAL_READ_CHUNK)
            if not chunk:
                continue

            frames = parser.feed(chunk)
            for frame in frames:
                scan = assembler.push_frame(frame)
                if scan is None:
                    continue

                # Нам обычно нужен только самый новый скан.
                # Поэтому очищаем очередь и кладём свежие данные.
                try:
                    while True:
                        q.get_nowait()
                except Empty:
                    pass

                try:
                    q.put_nowait(scan)
                except Full:
                    pass

                if EXPORT_CSV:
                    fname = time.strftime(
                        "ld06_scan_%Y%m%d_%H%M%S.csv",
                        time.localtime(scan.scan_ts_unix),
                    )
                    export_scan_csv(os.path.join(EXPORT_DIR, fname), scan)

            if time.time() - last_stat > 2.0:
                logger.info(
                    "stats: bytes_in=%d ok_frames=%d crc_errors=%d invalid_n=%d "
                    "resync_shifts=%d buffer_len=%d",
                    parser.bytes_in,
                    parser.frames_ok,
                    parser.crc_errors,
                    parser.invalid_n_points,
                    parser.resync_shifts,
                    len(parser.buf),
                )
                last_stat = time.time()


# ============================================================
# GUI / ВИЗУАЛИЗАЦИЯ
# ============================================================


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=[
            logging.FileHandler(LOG_FILE, encoding="utf-8"),
            logging.StreamHandler(sys.stdout),
        ],
    )
    logger = logging.getLogger("ld06")

    q: Queue = Queue(maxsize=1)
    stop_evt = threading.Event()

    th = threading.Thread(
        target=reader_loop,
        args=(q, stop_evt, logger),
        daemon=True,
    )
    th.start()

    # Настройка окна графика
    fig, ax = plt.subplots()
    ax.set_title("LD06 live XY (q=quit, s=save)")
    ax.set_xlim(-PLOT_SCALE_M, PLOT_SCALE_M)
    ax.set_ylim(-PLOT_SCALE_M, PLOT_SCALE_M)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_xlabel("X, m")
    ax.set_ylabel("Y, m")

    if USE_INTENSITY_COLORS:
        scat = ax.scatter(
            [],
            [],
            s=POINT_SIZE,
            c=[],
            cmap=COLORMAP,
            vmin=INTENSITY_MIN,
            vmax=INTENSITY_MAX,
            alpha=POINT_ALPHA,
        )
        cbar = fig.colorbar(scat, ax=ax)
        cbar.set_label("Intensity")
    else:
        scat = ax.scatter([], [], s=POINT_SIZE, alpha=POINT_ALPHA)

    text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        verticalalignment="top",
        horizontalalignment="left",
        bbox=dict(facecolor="white", alpha=0.75, edgecolor="none"),
    )

    last_scan: Optional[ScanBundle] = None
    saved_counter = 0

    def on_key(event):
        nonlocal saved_counter

        if event.key == "q":
            plt.close(fig)
        elif event.key == "s":
            saved_counter += 1
            fname = time.strftime(f"ld06_snapshot_%Y%m%d_%H%M%S_{saved_counter}.png")
            fig.savefig(fname, dpi=SNAPSHOT_DPI)
            print(f"Saved snapshot: {fname}")

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame_idx):
        nonlocal last_scan

        try:
            last_scan = q.get_nowait()
        except Empty:
            pass

        if last_scan is None:
            return scat, text

        xs = [p.x_m for p in last_scan.points]
        ys = [p.y_m for p in last_scan.points]

        scat.set_offsets(list(zip(xs, ys)))
        scat.set_sizes([POINT_SIZE] * len(xs))

        if USE_INTENSITY_COLORS:
            intensities = [p.intensity for p in last_scan.points]
            scat.set_array(intensities)

        hz = last_scan.speed_dps / 360.0 if last_scan.speed_dps else 0.0
        rpm = last_scan.speed_dps / 6.0 if last_scan.speed_dps else 0.0

        text.set_text(
            f"points={len(last_scan.points)}\n"
            f"speed={last_scan.speed_dps} deg/s\n"
            f"~{hz:.2f} Hz, ~{rpm:.1f} rpm\n"
            f"scale=±{PLOT_SCALE_M:.2f} m\n"
            f"point_size={POINT_SIZE}\n"
            f"offset={ANGLE_OFFSET_DEG:.1f} deg"
        )
        return scat, text

    _ani = FuncAnimation(fig, update, interval=ANIM_INTERVAL_MS, blit=False)

    try:
        plt.show()
    finally:
        stop_evt.set()
        th.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
