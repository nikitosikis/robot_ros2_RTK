#!/usr/bin/env python3
"""
FPS benchmark для semantic_inference (ade20k-hrnetv2-c1)
TensorRT 10.x, dynamic-shape engine.
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import tensorrt as trt

NUM_CLASSES  = 150
MEAN         = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD          = np.array([0.229, 0.224, 0.225], dtype=np.float32)
WARMUP_RUNS  = 30
BENCH_FRAMES = 100

np.random.seed(42)
PALETTE = np.random.randint(0, 255, size=(NUM_CLASSES, 3), dtype=np.uint8)

DEFAULT_H, DEFAULT_W = 512, 512


def preprocess(bgr_frame: np.ndarray, input_h: int, input_w: int) -> torch.Tensor:
    img = cv2.resize(bgr_frame, (input_w, input_h), interpolation=cv2.INTER_LINEAR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    img = (img - MEAN) / STD
    tensor = torch.from_numpy(img.transpose(2, 0, 1)[np.newaxis]).float()
    return tensor.to("cuda", non_blocking=True)


def colorize(label_map: np.ndarray, orig_h: int, orig_w: int) -> np.ndarray:
    color_bgr = PALETTE[label_map % NUM_CLASSES][:, :, ::-1]
    return cv2.resize(color_bgr, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)


class TrtBackend:
    def __init__(self, engine_path: str, input_h: int = DEFAULT_H, input_w: int = DEFAULT_W):
        logger  = trt.Logger(trt.Logger.WARNING)
        runtime = trt.Runtime(logger)
        with open(engine_path, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream  = torch.cuda.Stream()
        self.input_name = self.output_name = None
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_name = name
            else:
                self.output_name = name
        assert self.input_name  is not None, "Входной тензор не найден"
        assert self.output_name is not None, "Выходной тензор не найден"
        DTYPE_MAP = {
            trt.DataType.FLOAT: torch.float32, trt.DataType.HALF:  torch.float16,
            trt.DataType.INT32: torch.int32,   trt.DataType.INT8:  torch.int8,
            trt.DataType.BOOL:  torch.bool,
        }
        self.out_torch_dtype = DTYPE_MAP.get(
            self.engine.get_tensor_dtype(self.output_name), torch.float32)
        self.input_h, self.input_w = input_h, input_w
        input_shape = (1, 3, input_h, input_w)
        self.context.set_input_shape(self.input_name, input_shape)
        out_shape = tuple(self.context.get_tensor_shape(self.output_name))
        print(f"[TRT] движок    : {engine_path}")
        print(f"[TRT] вход      : '{self.input_name}'  {input_h} x {input_w}")
        print(f"[TRT] выход     : '{self.output_name}'  shape={out_shape}")
        self.input_buf  = torch.zeros(input_shape, dtype=torch.float32, device="cuda")
        self.output_buf = torch.zeros(out_shape,   dtype=self.out_torch_dtype, device="cuda")
        self.context.set_tensor_address(self.input_name,  self.input_buf.data_ptr())
        self.context.set_tensor_address(self.output_name, self.output_buf.data_ptr())

    def warmup(self):
        dummy = torch.zeros(
            (1, 3, self.input_h, self.input_w), dtype=torch.float32, device="cuda")
        for _ in range(WARMUP_RUNS):
            self._run(dummy)

    def _run(self, input_tensor: torch.Tensor) -> torch.Tensor:
        self.input_buf.copy_(input_tensor)
        with torch.cuda.stream(self.stream):
            self.context.execute_async_v3(stream_handle=self.stream.cuda_stream)
        self.stream.synchronize()
        return self.output_buf

    def infer(self, input_tensor: torch.Tensor) -> np.ndarray:
        out = self._run(input_tensor)
        if out.ndim == 4:
            return out[0].argmax(dim=0).cpu().numpy().astype(np.int32)
        return out[0].cpu().numpy().astype(np.int32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model",
        default=str(Path.home() / ".semantic_inference/ade20k-hrnetv2-c1.trt"))
    ap.add_argument("--video",  default=str(Path(__file__).parent / "test_video.mp4"))
    ap.add_argument("--height", type=int, default=DEFAULT_H)
    ap.add_argument("--width",  type=int, default=DEFAULT_W)
    ap.add_argument("--alpha",  type=float, default=0.5)
    args = ap.parse_args()

    assert torch.cuda.is_available(), "CUDA недоступна!"

    backend = TrtBackend(args.model, input_h=args.height, input_w=args.width)
    input_h, input_w = backend.input_h, backend.input_w

    print(f"Прогрев ({WARMUP_RUNS} кадров)...")
    backend.warmup()
    print(f"Замер ({BENCH_FRAMES} кадров)...")

    cap = cv2.VideoCapture(args.video)
    assert cap.isOpened(), f"Не удалось открыть видео: {args.video}"

    frame_count  = 0
    total_time   = 0.0

    while frame_count < BENCH_FRAMES:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        orig_h, orig_w = frame.shape[:2]

        t0        = time.perf_counter()
        tensor    = preprocess(frame, input_h, input_w)
        label_map = backend.infer(tensor)
        total_time += time.perf_counter() - t0

        frame_count += 1
        print(f"\rОбработано кадров: {frame_count}/{BENCH_FRAMES}", end="", flush=True)

        # ── ВИЗУАЛИЗАЦИЯ: закомментируй блок ниже, чтобы отключить окно ──────
        color_mask = colorize(label_map, orig_h, orig_w)
        overlay    = cv2.addWeighted(frame, 1.0 - args.alpha, color_mask, args.alpha, 0)
        cv2.imshow("semantic_inference TRT", overlay)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        # ── конец блока визуализации ──────────────────────────────────────────

    cap.release()
    cv2.destroyAllWindows()

    avg_fps = BENCH_FRAMES / total_time
    print(f"\n{'─'*40}")
    print(f"Среднее время на кадр : {total_time/BENCH_FRAMES*1000:.2f} мс")
    print(f"Средний FPS           : {avg_fps:.2f}")
    print(f"{'─'*40}")


if __name__ == "__main__":
    main()
