#!/usr/bin/env python3
"""YOLOv8n-seg FPS benchmark — RKNN (Orange Pi / RK3588)"""

import cv2
import numpy as np
import time
from rknn.api import RKNN
from pathlib import Path

package_path = Path(__file__).parent
MODEL_PATH   = str(package_path / "yolov8n-seg-rk3588.rknn")
VIDEO_PATH   = str(package_path / "test_video.mp4")
CONF_THRES   = 0.5
IOU_THRES    = 0.45
INPUT_W      = 640
INPUT_H      = 640
WARMUP_RUNS  = 30
BENCH_FRAMES = 100

COCO_CLASSES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck','boat',
    'traffic light','fire hydrant','stop sign','parking meter','bench','bird','cat',
    'dog','horse','sheep','cow','elephant','bear','zebra','giraffe','backpack',
    'umbrella','handbag','tie','suitcase','frisbee','skis','snowboard','sports ball',
    'kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket',
    'bottle','wine glass','cup','fork','knife','spoon','bowl','banana','apple',
    'sandwich','orange','broccoli','carrot','hot dog','pizza','donut','cake',
    'chair','couch','potted plant','bed','dining table','toilet','tv','laptop',
    'mouse','remote','keyboard','cell phone','microwave','oven','toaster','sink',
    'refrigerator','book','clock','vase','scissors','teddy bear','hair drier','toothbrush'
]

np.random.seed(42)
COLORS = np.random.randint(0, 255, size=(80, 3), dtype=np.uint8)


def preprocess(img):
    h, w  = img.shape[:2]
    scale = min(INPUT_W / w, INPUT_H / h)
    new_w, new_h = int(w * scale), int(h * scale)
    padded = cv2.copyMakeBorder(
        cv2.resize(img, (new_w, new_h)),
        (INPUT_H - new_h) // 2, (INPUT_H - new_h + 1) // 2,
        (INPUT_W - new_w) // 2, (INPUT_W - new_w + 1) // 2,
        cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    tensor = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB).astype(np.uint8)
    top    = (INPUT_H - new_h) // 2
    left   = (INPUT_W - new_w) // 2
    return tensor, (top, left, scale, w, h)


def postprocess(outputs, pad_info):
    dets = np.squeeze(outputs[0])
    if dets.ndim != 2:
        return [], [], [], []
    if dets.shape[0] == 116:
        dets = dets.T
    proto        = np.squeeze(outputs[1])
    nc           = dets.shape[1] - 36
    scores       = np.max(dets[:, 4:4+nc], axis=1)
    class_ids    = np.argmax(dets[:, 4:4+nc], axis=1)
    mask_coeffs  = dets[:, 4+nc:]
    keep         = scores >= CONF_THRES
    dets, scores, class_ids, mask_coeffs = (
        dets[keep], scores[keep], class_ids[keep], mask_coeffs[keep])
    if len(scores) == 0:
        return [], [], [], []
    bxy   = dets[:, :2]; bwh = dets[:, 2:4]
    boxes = np.hstack([bxy - bwh/2, bxy + bwh/2]).astype(np.float32)
    idx   = cv2.dnn.NMSBoxes(
        np.hstack([bxy - bwh/2, bwh]).tolist(), scores.tolist(), CONF_THRES, IOU_THRES)
    if len(idx) == 0:
        return [], [], [], []
    idx   = idx.flatten()
    boxes, scores, class_ids, mask_coeffs = (
        boxes[idx], scores[idx], class_ids[idx], mask_coeffs[idx])
    top, left, scale, orig_w, orig_h = pad_info
    masks = []
    for i in range(len(boxes)):
        raw  = (mask_coeffs[i] @ proto.reshape(32, -1))
        raw  = (1.0 / (1.0 + np.exp(-raw))).reshape(160, 160)
        m    = cv2.resize((raw > 0.5).astype(np.uint8)*255, (INPUT_W, INPUT_H),
                          interpolation=cv2.INTER_NEAREST)
        x1b, y1b, x2b, y2b = boxes[i].astype(np.int32)
        crop = np.zeros((INPUT_H, INPUT_W), dtype=np.uint8)
        crop[y1b:y2b, x1b:x2b] = m[y1b:y2b, x1b:x2b]
        ph, pw = int(orig_h * scale), int(orig_w * scale)
        masks.append(cv2.resize(crop[top:top+ph, left:left+pw], (orig_w, orig_h),
                                interpolation=cv2.INTER_NEAREST))
    boxes[:, [0,2]] = (boxes[:, [0,2]] - left) / scale
    boxes[:, [1,3]] = (boxes[:, [1,3]] - top)  / scale
    boxes = np.clip(boxes, 0, [orig_w, orig_h, orig_w, orig_h]).astype(np.int32)
    return boxes, scores, class_ids, masks


def draw(img, boxes, scores, class_ids, masks):
    out = img.copy()
    ov  = np.zeros_like(img)
    for m, cid in zip(masks, class_ids):
        ov[m > 0] = COLORS[cid % 80].tolist()
    out = cv2.addWeighted(out, 0.6, ov, 0.4, 0)
    for box, score, cid in zip(boxes, scores, class_ids):
        x1, y1, x2, y2 = box
        c = COLORS[cid % 80].tolist()
        cv2.rectangle(out, (x1, y1), (x2, y2), c, 2)
        cv2.putText(out, f"{COCO_CLASSES[cid]}: {score:.2f}", (x1, max(y1-5, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)
    return out


rknn = RKNN()
print(f"Загрузка модели: {MODEL_PATH}")
assert rknn.load_rknn(MODEL_PATH)     == 0, "Не удалось загрузить RKNN модель"
assert rknn.init_runtime(target='rk3588') == 0, "Не удалось инициализировать RKNN runtime"

print(f"Прогрев ({WARMUP_RUNS} кадров)...")
dummy = np.zeros((INPUT_H, INPUT_W, 3), dtype=np.uint8)
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

    t0                             = time.perf_counter()
    tensor, pad_info               = preprocess(frame)
    outputs                        = rknn.inference(inputs=[tensor])
    boxes, scores, class_ids, masks = postprocess(outputs, pad_info)
    total_time                    += time.perf_counter() - t0

    frame_count += 1
    print(f"\rОбработано кадров: {frame_count}/{BENCH_FRAMES}", end="", flush=True)

    # ── ВИЗУАЛИЗАЦИЯ: закомментируй блок ниже, чтобы отключить окно ──────────
    #result = draw(frame, boxes, scores, class_ids, masks)
    #cv2.imshow("YOLOv8n-seg RKNN", result)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
        #break
    # ── конец блока визуализации ──────────────────────────────────────────────

cap.release()
cv2.destroyAllWindows()
rknn.release()

avg_fps = BENCH_FRAMES / total_time
print(f"\n{'─'*40}")
print(f"Среднее время на кадр : {total_time/BENCH_FRAMES*1000:.2f} мс")
print(f"Средний FPS           : {avg_fps:.2f}")
print(f"{'─'*40}")
