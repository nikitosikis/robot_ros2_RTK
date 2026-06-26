#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from rknn.api import RKNN
from pathlib import Path

# COCO class names (80 classes)
COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
    'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

# COCO class id -> ADE20K label id (0 = background).
COCO_TO_ADE20K = {
    # люди
    0:  12,   # person -> person

    # сидячие объекты / мебель
    13: 69,   # bench -> bench
    56: 19,   # chair -> chair
    57: 23,   # couch -> sofa
    59: 7,    # bed -> bed
    60: 15,   # dining table -> table
    61: 65,   # toilet -> toilet
    58: 17,   # potted plant -> plant

    # посуда и тара
    39: 98,   # bottle -> bottle
    40: 147,  # wine glass -> drinking glass
    41: 147,  # cup -> drinking glass
    #45: 125,  # bowl -> pot

    # техника / экраны
    62: 89,   # tv -> television
    63: 74,   # laptop -> computer
    #68: 124,  # microwave -> microwave
    #69: 118,  # oven -> oven
    #71: 47,   # sink -> sink
    #72: 50,   # refrigerator -> refrigerator

    # книги / декор
    73: 67,   # book -> book
    74: 148,  # clock -> clock
    75: 135,  # vase -> vase

    # еда
    46: 120, 47: 120, 48: 120, 49: 120, 50: 120,
    51: 120, 52: 120, 53: 120, 54: 120, 55: 120,

    # сумки / одежда
    #24: 115, 26: 115, 28: 115,
    #27: 92,

}

# ADE20K ID для визуализации фона на отладочном изображении
ADE20K_COLORS = {
    3: (255, 0, 0),  # floor
    }

class YOLOSegmentationNode(Node):
    def __init__(self):
        super().__init__('yolo_segmentation_node')

        # ── Базовые параметры ──────────────────────────────────────────────
        self.declare_parameter('model_path', 'yolov8n-seg_rknn_model/yolov8n-seg-rk3588.rknn')
        self.declare_parameter('input_topic',  '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic',  '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('depth_info_topic', '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('confidence_threshold', 0.2)
        self.declare_parameter('iou_threshold', 0.35)
        self.declare_parameter('input_width',  640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('label_topic',  '/yolo_segmentation/labels')

        # ── Геометрия сцены (измерить рулеткой!) ──────────────────────────
        # Высота оптического центра камеры над полом, метры
        self.declare_parameter('h_floor_m',    0.23)
        
        # Допустимое отклонение ±, метры
        self.declare_parameter('h_tolerance',  0.15)

        # ── Считываем параметры ────────────────────────────────────────────
        input_topic       = self.get_parameter('input_topic').value
        depth_topic       = self.get_parameter('depth_topic').value
        depth_info_topic  = self.get_parameter('depth_info_topic').value
        self.conf_thres   = self.get_parameter('confidence_threshold').value
        self.iou_thres    = self.get_parameter('iou_threshold').value
        self.input_width  = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.label_topic  = self.get_parameter('label_topic').value

        self.h_floor   = self.get_parameter('h_floor_m').value
        self.h_tol     = self.get_parameter('h_tolerance').value

        # ── Инициализация RKNN ────────────────────────────────────────────
        self.model_path = '/home/ubuntu24/ros2_yolo_ws/src/yolo_segmentation/models/yolov8n-seg-rk3588.rknn'
        self.rknn = RKNN()
        self.get_logger().info(f'Loading RKNN model: {self.model_path}')
        if self.rknn.load_rknn(self.model_path) != 0:
            self.get_logger().error('Load RKNN model failed!')
            rclpy.shutdown(); return
        if self.rknn.init_runtime(target='rk3588') != 0:
            self.get_logger().error('Init RKNN runtime failed!')
            rclpy.shutdown(); return

        self.bridge = CvBridge()

        # Кэш последнего depth-кадра и параметров камеры
        self.latest_depth: np.ndarray | None = None
        self.cam_fy: float | None = None
        self.cam_cy: float | None = None

        # ── Подписки и публикации ─────────────────────────────────────────
        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, depth_info_topic, self.camera_info_callback, 10)

        self.publisher       = self.create_publisher(Image, '/yolo_segmentation/result', 10)
        self.label_publisher = self.create_publisher(Image, self.label_topic, 10)

        self.get_logger().info(
            f'Node started.\n'
            f'  RGB   : {input_topic}\n'
            f'  Depth : {depth_topic}\n'
            f'  Labels: {self.label_topic}\n'
            f'  h_floor={self.h_floor} m  tol=±{self.h_tol} m'
        )

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks для depth
    # ──────────────────────────────────────────────────────────────────────

    def camera_info_callback(self, msg: CameraInfo):
        """Получаем интринсики камеры глубины один раз."""
        if self.cam_fy is None:
            self.cam_fy = msg.k[4]   # K[1,1]
            self.cam_cy = msg.k[5]   # K[1,2]
            self.get_logger().info(
                f'Camera intrinsics received: fy={self.cam_fy:.2f}, cy={self.cam_cy:.2f}')
            # После получения интринсиков подписка больше не нужна
            self.destroy_subscription(self.depth_info_sub)

    def depth_callback(self, msg: Image):
        """Кешируем depth-изображение (16UC1, мм → float32, м)."""
        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.latest_depth = depth_raw.astype(np.float32) / 1000.0  # мм → м

    # ──────────────────────────────────────────────────────────────────────
    # Классификация фона по глубине
    # ──────────────────────────────────────────────────────────────────────

    def classify_background(self, shape: tuple) -> np.ndarray:
        """
        Возвращает label image (int32) с метками стен/пола/потолка.

        Принцип: камера горизонтальна → ось Y кадра совпадает с вертикалью мира.
        Y_3D = (v - cy) * d / fy
          Y_3D > 0  — ниже камеры (пол при Y_3D ≈ h_floor)
        """
        h, w = shape[:2]
        # По умолчанию всё — стена (ADE20K id=1)
        label_bg = np.zeros((h, w), dtype=np.int32)

        if self.latest_depth is None or self.cam_fy is None:
            return label_bg

        depth = self.latest_depth

        # Если размер depth не совпадает с RGB — масштабируем
        if depth.shape[0] != h or depth.shape[1] != w:
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

        # Сетка строк пикселей (H, W)
        rows = np.tile(np.arange(h, dtype=np.float32).reshape(h, 1), (1, w))

        valid = depth > 0.1  # отбрасываем невалидные пиксели

        # 3D координата Y в системе камеры (вниз — положительная)
        Y3D = np.where(valid, (rows - self.cam_cy) * depth / self.cam_fy, np.nan)

        # Пол: Y_3D ∈ [h_floor, h_floor + tol]
        floor_mask = valid & (Y3D >= self.h_floor) \
                           & (Y3D <= self.h_floor + self.h_tol)

        label_bg[floor_mask] = 3   # ADE20K: floor

        return label_bg

    # ──────────────────────────────────────────────────────────────────────
    # Препроцессинг / постпроцессинг YOLO
    # ──────────────────────────────────────────────────────────────────────

    def preprocess(self, img):
        """Letterbox resize + padding до размера модели."""
        h, w = img.shape[:2]
        scale = min(self.input_width / w, self.input_height / h)
        new_w, new_h = int(w * scale), int(h * scale)

        img_resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        dw = self.input_width  - new_w
        dh = self.input_height - new_h
        top, bottom = dh // 2, dh - dh // 2
        left, right = dw // 2, dw - dw // 2
        img_padded = cv2.copyMakeBorder(
            img_resized, top, bottom, left, right,
            cv2.BORDER_CONSTANT, value=(114, 114, 114))

        img_input = cv2.cvtColor(img_padded, cv2.COLOR_BGR2RGB).astype(np.uint8)
        pad_info = (top, left, scale, w, h)
        return img_input, pad_info

    def postprocess(self, outputs, pad_info):
        """Декодирование детекций и масок."""
        dets = np.squeeze(outputs[0])
        if dets.ndim != 2:
            self.get_logger().error(f'Unexpected dets shape: {dets.shape}')
            return np.empty((0, 4), dtype=np.int32), np.empty(0), np.empty(0), []

        if dets.shape[0] == 116:
            dets = dets.T  # → (N, 116)

        prototypes = np.squeeze(outputs[1])  # (32, 160, 160)

        num_classes = dets.shape[1] - 4 - 32
        boxes_xywh  = dets[:, :4]
        class_logits = dets[:, 4:4 + num_classes]
        mask_coeffs  = dets[:, 4 + num_classes:]

        scores    = np.max(class_logits, axis=1)
        class_ids = np.argmax(class_logits, axis=1)

        keep = scores >= self.conf_thres
        boxes_xywh  = boxes_xywh[keep]
        scores      = scores[keep]
        class_ids   = class_ids[keep]
        mask_coeffs = mask_coeffs[keep]

        if len(boxes_xywh) == 0:
            return np.empty((0, 4), dtype=np.int32), np.empty(0), np.empty(0), []

        x1 = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2.0
        y1 = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2.0
        x2 = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2.0
        y2 = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2.0
        boxes_640 = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)

        boxes_nms = np.hstack([boxes_640[:, :2], boxes_640[:, 2:] - boxes_640[:, :2]])
        indices = cv2.dnn.NMSBoxes(
            boxes_nms.tolist(), scores.tolist(), self.conf_thres, self.iou_thres)
        if len(indices) == 0:
            return np.empty((0, 4), dtype=np.int32), np.empty(0), np.empty(0), []
        indices = indices.flatten()

        boxes_640   = boxes_640[indices]
        scores      = scores[indices]
        class_ids   = class_ids[indices]
        mask_coeffs = mask_coeffs[indices]

        top_pad, left_pad, scale, orig_w, orig_h = pad_info
        prototypes_flat = prototypes.reshape(32, -1)  # (32, 25600)
        masks_original = []

        for i in range(len(boxes_640)):
            coeff    = mask_coeffs[i]
            mask_raw = coeff @ prototypes_flat
            mask_raw = 1.0 / (1.0 + np.exp(-mask_raw))
            mask_raw = (mask_raw.reshape(160, 160) > 0.5).astype(np.uint8) * 255

            mask_640 = cv2.resize(
                mask_raw, (self.input_width, self.input_height),
                interpolation=cv2.INTER_NEAREST)

            x1b, y1b, x2b, y2b = boxes_640[i].astype(np.int32)
            mask_cropped = np.zeros((self.input_height, self.input_width), dtype=np.uint8)
            mask_cropped[y1b:y2b, x1b:x2b] = mask_640[y1b:y2b, x1b:x2b]

            padded_h = int(orig_h * scale)
            padded_w = int(orig_w * scale)
            mask_no_pad = mask_cropped[
                top_pad:top_pad + padded_h,
                left_pad:left_pad + padded_w]

            mask_original = cv2.resize(
                mask_no_pad, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)
            masks_original.append(mask_original)

        boxes_original = boxes_640.copy()
        boxes_original[:, [0, 2]] = (boxes_original[:, [0, 2]] - left_pad) / scale
        boxes_original[:, [1, 3]] = (boxes_original[:, [1, 3]] - top_pad) / scale
        boxes_original = np.clip(
            boxes_original, 0, [orig_w, orig_h, orig_w, orig_h]).astype(np.int32)

        return boxes_original, scores, class_ids, masks_original

    # ──────────────────────────────────────────────────────────────────────
    # Визуализация
    # ──────────────────────────────────────────────────────────────────────

    def draw_segmentation(self, img, boxes, scores, class_ids, masks, label_img):

        #Отладочная картинка: полупрозрачные маски объектов + цветной фон.
        #Яркость исходного изображения вне масок не меняется.
     
        result = img.copy()
        overlay = np.zeros_like(img, dtype=np.uint8)

        # Закрашиваем фон по label_img
        for ade_id, color in ADE20K_COLORS.items():
            overlay[label_img == ade_id] = color

        # Объектные маски поверх фона
        np.random.seed(42)
        obj_colors = np.random.randint(0, 255, size=(80, 3), dtype=np.uint8)
        for mask, cls_id in zip(masks, class_ids):
            overlay[mask > 0] = obj_colors[cls_id % 80].tolist()

        # Смешиваем только там, где реально есть маска/цвет
        blended = cv2.addWeighted(img, 0.55, overlay, 0.45, 0)
        mask_exists = np.any(overlay > 0, axis=2, keepdims=True)
        result = np.where(mask_exists, blended, img)

        # Рамки и подписи
        for box, score, cls_id in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = box
            color = obj_colors[cls_id % 80].tolist()
            cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
            name = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else f'cls{cls_id}'
            cv2.putText(result, f'{name}:{score:.2f}', (x1, max(y1 - 5, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        return result

    # ──────────────────────────────────────────────────────────────────────
    # Формирование label image
    # ──────────────────────────────────────────────────────────────────────

    def generate_label_image(self, masks, class_ids, shape):
        """
        Итоговый label image для Hydra (32SC1, ADE20K IDs).

        Порядок наложения:
          1. Геометрический фон (стена=0, пол=3, потолок=5) — по depth
          2. Объекты YOLO — перекрывают фон
        """
        # Шаг 1: геометрический фон
        label_img = self.classify_background(shape)

        # Шаг 2: объекты от YOLO поверх
        for mask, cls_id in zip(masks, class_ids):
            ade_id = COCO_TO_ADE20K.get(int(cls_id), 0)
            if ade_id == 0:
                continue
            label_img[mask > 0] = ade_id

        return label_img

    # ──────────────────────────────────────────────────────────────────────
    # Главный callback
    # ──────────────────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        input_tensor, pad_info = self.preprocess(cv_image)

        try:
            outputs = self.rknn.inference(inputs=[input_tensor])
        except Exception as e:
            self.get_logger().error(f'RKNN inference failed: {e}')
            return

        boxes, scores, class_ids, masks = self.postprocess(outputs, pad_info)

        # Label image (основной выход для Hydra)
        label_img = self.generate_label_image(masks, class_ids, cv_image.shape)

        label_msg = self.bridge.cv2_to_imgmsg(label_img, encoding='32SC1')
        label_msg.header = msg.header
        self.label_publisher.publish(label_msg)

        # Визуализация (отладка)
        result_img = self.draw_segmentation(cv_image, boxes, scores, class_ids, masks, label_img)
        result_msg = self.bridge.cv2_to_imgmsg(result_img, encoding='bgr8')
        result_msg.header = msg.header
        self.publisher.publish(result_msg)

        self.get_logger().debug(
            f'Published: {len(boxes)} objects | '
            f'depth_ok={self.latest_depth is not None} | '
            f'intrinsics_ok={self.cam_fy is not None}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLOSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
