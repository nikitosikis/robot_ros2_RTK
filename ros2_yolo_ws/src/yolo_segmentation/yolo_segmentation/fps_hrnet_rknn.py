#!/usr/bin/env python3
"""
FPS benchmark для semantic_inference (ade20k-hrnetv2-c1)
RKNN, NPU Rockchip RK3588 (Orange Pi 5 Max).
"""

import cv2
import numpy as np
import time
from rknn.api import RKNN
from pathlib import Path

package_path = Path(__file__).parent
MODEL_PATH   = str(package_path / "ade20k-hrnetv2-c1.rknn")
VIDEO_PATH   = str(package_path / "test_video.mp4")

NUM_CLASSES  = 150
MEAN         = np.array([123.675, 116.28, 103.53], dtype=np.float32)
STD          = np.array([58.395,  57.12,  57.375],  dtype=np.float32)
INPUT_H      = 512
INPUT_W      = 512
WARMUP_RUNS  = 30
BENCH_FRAMES = 100
ALPHA        = 0.5

np.random.seed(42)
PALETTE = np.random.randint(0, 255, size=(NUM_CLASSES, 3), dtype=np.uint8)


def preprocess(bgr_frame: np.ndarray) -> np.ndarray:
    img = cv2.resize(bgr_frame, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
    img = (img - MEAN) / STD
    return img.transpose(2, 0, 1)[np.newaxis].astype(np.float32)  # NCHW


def colorize(label_map: np.ndarray, orig_h: int, orig_w: int) -> np.ndarray:
    color_bgr = PALETTE[label_map % NUM_CLASSES][:, :, ::-1]
    return cv2.resize(color_bgr, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)


rknn = RKNN()
print(f"Загрузка модели: {MODEL_PATH}")
assert rknn.load_rknn(MODEL_PATH) == 0, "Не удалось загрузить RKNN модель"
assert rknn.init_runtime(target='rk3588') == 0, "Не удалось инициализировать RKNN runtime"

print(f"Прогрев ({WARMUP_RUNS} кадров)...")
dummy = np.zeros((1, 3, INPUT_H, INPUT_W), dtype=np.float32)
for _ in range(WARMUP_RUNS):
    rknn.inference(inputs=[dummy])

print(f"Замер ({BENCH_FRAMES} кадров)...")

cap = cv2.VideoCapture(VIDEO_PATH)
assert cap.isOpened(), f"Не удалось открыть: {VIDEO_PATH}"

frame_count = 0
total_time  = 0.0

while frame_count < BENCH_FRAMES:
    ret, frame = cap.read()
    if not ret:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        continue

    orig_h, orig_w = frame.shape[:2]

    t0      = time.perf_counter()
    inp     = preprocess(frame)
    outputs = rknn.inference(inputs=[inp])
    out     = outputs[0]  # (1, NUM_CLASSES, H, W) или (1, H, W)
    if out.ndim == 4:
        label_map = out[0].argmax(axis=0).astype(np.int32)
    else:
        label_map = out[0].astype(np.int32)
    total_time += time.perf_counter() - t0

    frame_count += 1
    print(f"\rОбработано кадров: {frame_count}/{BENCH_FRAMES}", end="", flush=True)

    # ── ВИЗУАЛИЗАЦИЯ: закомментируй блок ниже, чтобы отключить окно ──────────
    color_mask = colorize(label_map, orig_h, orig_w)
    overlay    = cv2.addWeighted(frame, 1.0 - ALPHA, color_mask, ALPHA, 0)
    cv2.imshow("HRNet RKNN", overlay)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    # ── конец блока визуализации ──────────────────────────────────────────────

cap.release()
cv2.destroyAllWindows()
rknn.release()

avg_fps = frame_count / total_time
print(f"\n{'─'*42}")
print(f"Среднее время на кадр : {total_time/frame_count*1000:.2f} мс")
print(f"Средний FPS           : {avg_fps:.2f}")
print(f"{'─'*42}")
