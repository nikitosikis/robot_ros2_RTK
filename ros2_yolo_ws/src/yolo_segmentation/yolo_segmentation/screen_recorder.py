#!/usr/bin/env python3
"""
Запись экрана через Pillow + ImageGrab + cv2.
Зависимости (всё через apt, без venv):
    sudo apt install python3-pil python3-opencv scrot

Запуск:
    python3 screen_recorder.py
    python3 screen_recorder.py --fps 10
    python3 screen_recorder.py --output my_video.mp4
Остановка: Ctrl+C
"""

import argparse
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


def get_monitor_hz() -> float:
    try:
        import subprocess
        out = subprocess.check_output(['xrandr'], text=True)
        for line in out.splitlines():
            if '*' in line:
                for token in line.split():
                    if '*' in token:
                        hz = float(token.replace('*', '').replace('+', ''))
                        return hz
    except Exception:
        pass
    return 60.0


def make_grabber():
    try:
        from PIL import ImageGrab
        def grab():
            img = ImageGrab.grab()
            return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        grab()
        print('Бэкенд захвата: PIL ImageGrab')
        return grab
    except Exception as e:
        print(f'PIL ImageGrab недоступен: {e}')

    try:
        from Xlib import display as xdisplay, X
        dsp = xdisplay.Display()
        root = dsp.screen().root
        geo = root.get_geometry()
        W, H = geo.width, geo.height
        def grab():
            raw = root.get_image(0, 0, W, H, X.ZPixmap, 0xFFFFFFFF)
            img = np.frombuffer(raw.data, dtype=np.uint8).reshape(H, W, 4)
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        grab()
        print('Бэкенд захвата: python3-xlib')
        return grab
    except Exception as e:
        print(f'python3-xlib недоступен: {e}')

    print('Ошибка: не найден ни один бэкенд захвата экрана.')
    print('Установи хотя бы один из:\n  sudo apt install python3-pil\n  sudo apt install python3-xlib')
    sys.exit(1)


def get_screen_size(grab) -> tuple:
    frame = grab()
    h, w = frame.shape[:2]
    return w, h


def parse_args():
    parser = argparse.ArgumentParser(description='Запись экрана')
    parser.add_argument('--output', type=str, default=None,
                        help='Имя выходного файла (по умолчанию — дата/время)')
    parser.add_argument('--fps', type=float, default=None,
                        help='FPS (по умолчанию — частота обновления монитора)')
    return parser.parse_args()


def main():
    args = parse_args()

    grab = make_grabber()

    fps = args.fps if args.fps else get_monitor_hz()
    fps = max(1.0, fps)
    period = 1.0 / fps

    output_path = args.output
    if output_path is None:
        ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        output_path = f'screen_{ts}.mp4'

    width, height = get_screen_size(grab)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    if not writer.isOpened():
        print(f'Ошибка: не удалось открыть файл {output_path}')
        sys.exit(1)

    print(f'Запись: {width}x{height} @ {fps:.1f} FPS → {output_path}')
    print('Остановка: Ctrl+C')

    stop = False

    def handle_signal(sig, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    frame_count = 0
    start_time = time.monotonic()

    while not stop:
        t0 = time.monotonic()

        frame = grab()
        writer.write(frame)
        frame_count += 1

        elapsed = time.monotonic() - t0
        sleep_for = period - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)

    writer.release()

    total_time = time.monotonic() - start_time
    actual_fps = frame_count / total_time if total_time > 0 else 0
    size_mb = Path(output_path).stat().st_size / (1024 * 1024)

    print(f'\nЗапись завершена.')
    print(f'  Кадров записано : {frame_count}')
    print(f'  Длительность    : {total_time:.1f} с')
    print(f'  Средний FPS     : {actual_fps:.1f}')
    print(f'  Размер файла    : {size_mb:.1f} МБ')
    print(f'  Файл            : {output_path}')


if __name__ == '__main__':
    main()