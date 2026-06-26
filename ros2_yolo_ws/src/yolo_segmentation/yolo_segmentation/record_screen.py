import subprocess
import sys
import signal
from pathlib import Path

OUTPUT = Path("screen_record.mp4")
FPS = "15"          # меньше fps = меньше нагрузка
CRF = "28"          # больше значение = меньше размер и нагрузка, но хуже качество

def build_ffmpeg_cmd():
    if sys.platform.startswith("win"):
        # Windows
        return [
            "ffmpeg", "-y",
            "-f", "gdigrab",
            "-framerate", FPS,
            "-i", "desktop",
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-crf", CRF,
            "-pix_fmt", "yuv420p",
            str(OUTPUT),
        ]

    elif sys.platform.startswith("linux"):
        # Linux (X11)
        display = ":0.0"
        return [
            "ffmpeg", "-y",
            "-f", "x11grab",
            "-framerate", FPS,
            "-i", display,
            "-c:v", "libx264",
            "-preset", "ultrafast",
            "-crf", CRF,
            "-pix_fmt", "yuv420p",
            str(OUTPUT),
        ]

    else:
        raise RuntimeError("Этот пример рассчитан на Windows или Linux.")

def main():
    cmd = build_ffmpeg_cmd()
    proc = subprocess.Popen(cmd)

    def stop(*_):
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print("Запись экрана началась. Нажмите Ctrl+C, чтобы остановить.")
    try:
        proc.wait()
    finally:
        stop()

if __name__ == "__main__":
    main()