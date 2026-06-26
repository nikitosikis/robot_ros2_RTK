#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
    # люди / транспорт
    0: 12,   # person -> person
    1: 127,  # bicycle -> bicycle

    # сидячие объекты / мебель
    13: 69,  # bench -> bench
    56: 19,  # chair -> chair
    57: 23,  # couch -> sofa
    59: 7,   # bed -> bed
    60: 15,  # dining table -> table
    61: 65,  # toilet -> toilet
    58: 17,  # potted plant -> plant
    
    # посуда и тара
    39: 98,  # bottle -> bottle
    40: 148, # wine glass -> drinking glass
    41: 148, # cup -> drinking glass
    45: 125, # bowl -> pot

    # техника / экраны
    62: 89,  # tv -> television 
    63: 74,  # laptop -> computer
    68: 124, # microwave -> microwave
    69: 118, # oven -> oven
    71: 47,  # sink -> sink
    72: 50,  # refrigerator -> refrigerator

    # книги / декор
    73: 67,  # book -> book
    74: 149, # clock -> clock
    75: 135, # vase -> vase
    
    # еда
    46: 120, 47: 120, 48: 120, 49: 120, 50: 120,
    51: 120, 52: 120, 53: 120, 54: 120, 55: 120,

    # сумки / одежда
    24: 115, 26: 115, 28: 115,
    27: 92,

    # животные — общий класс animal
    14: 126, 15: 126, 16: 126, 17: 126, 18: 126,
    19: 126, 20: 126, 21: 126, 22: 126, 23: 126,

    # игрушки
    29: 119, 32: 119, 77: 108,
}

class YOLOSegmentationNode(Node):
    def __init__(self):
        super().__init__('yolo_segmentation_node')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n-seg_rknn_model/yolov8n-seg-rk3588.rknn')
        self.declare_parameter('input_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.4)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        self.declare_parameter('label_topic', '/yolo_segmentation/labels')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_thres = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.input_width = self.get_parameter('input_width').get_parameter_value().integer_value
        self.input_height = self.get_parameter('input_height').get_parameter_value().integer_value
        self.label_topic = self.get_parameter('label_topic').get_parameter_value().string_value

        # Resolve model path relative to package
        package_path = Path(__file__).parent.parent
        self.model_path = '/home/ubuntu24/ros2_yolo_ws/src/yolo_segmentation/models/yolov8n-seg-rk3588.rknn'

        # Инициализация RKNN
        self.rknn = RKNN()
        self.get_logger().info(f"Loading RKNN model: {self.model_path}")
        ret = self.rknn.load_rknn(self.model_path)
        if ret != 0:
            self.get_logger().error('Load RKNN model failed!')
            rclpy.shutdown()
            return

        self.get_logger().info('Init RKNN runtime (rk3588)...')
        ret = self.rknn.init_runtime(target='rk3588')
        if ret != 0:
            self.get_logger().error('Init RKNN runtime failed!')
            rclpy.shutdown()
            return

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(Image, input_topic, self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/yolo_segmentation/result', 10)
        self.label_publisher = self.create_publisher(Image, self.label_topic, 10)

        self.get_logger().info(
            f"Node started. Subscribing to {input_topic}, "
            f"publishing to /yolo_segmentation/result and {self.label_topic}"
        )

    def preprocess(self, img):
        """Letterbox resize + padding to model input size."""
        h, w = img.shape[:2]
        target_w, target_h = self.input_width, self.input_height
        scale = min(target_w / w, target_h / h)
        new_w, new_h = int(w * scale), int(h * scale)

        img_resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        dw = target_w - new_w
        dh = target_h - new_h
        top, bottom = dh // 2, dh - dh // 2
        left, right = dw // 2, dw - dw // 2
        img_padded = cv2.copyMakeBorder(
            img_resized, top, bottom, left, right,
            cv2.BORDER_CONSTANT, value=(114, 114, 114)
        )

        # RKNN: RGB, uint8, NHWC
        img_rgb = cv2.cvtColor(img_padded, cv2.COLOR_BGR2RGB)
        img_input = img_rgb.astype(np.uint8)

        pad_info = (top, left, scale, w, h)
        return img_input, pad_info

    def postprocess(self, outputs, pad_info):
        """Decode detections and masks."""
        # outputs[0]: (1, 116, N) или (1, N, 116)
        # outputs[1]: (1, 32, 160, 160)
        dets = np.squeeze(outputs[0])
        if dets.ndim != 2:
            self.get_logger().error(f"Unexpected dets shape: {dets.shape}")
            return (np.empty((0, 4), dtype=np.int32),
                    np.empty(0), np.empty(0), [])

        # Приводим к (N, 116)
        if dets.shape[0] == 116:
            dets = dets.T

        prototypes = np.squeeze(outputs[1])  # (32, 160, 160)

        total_features = dets.shape[1]
        num_classes = total_features - 4 - 32  # 4 bbox + 32 mask coefficients

        boxes_xywh = dets[:, :4]  # (N, 4)
        class_logits = dets[:, 4:4 + num_classes]  # (N, num_classes)
        mask_coeffs = dets[:, 4 + num_classes:]    # (N, 32)

        scores = np.max(class_logits, axis=1)
        class_ids = np.argmax(class_logits, axis=1)

        # Confidence filter
        keep = scores >= self.conf_thres
        boxes_xywh = boxes_xywh[keep]
        scores = scores[keep]
        class_ids = class_ids[keep]
        mask_coeffs = mask_coeffs[keep]

        if len(boxes_xywh) == 0:
            return (np.empty((0, 4), dtype=np.int32),
                    np.empty(0), np.empty(0), [])

        # Convert xywh (normalized / model space) to xyxy in model input space
        x1 = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2.0
        y1 = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2.0
        x2 = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2.0
        y2 = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2.0
        boxes_640 = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)

        # NMS expects (x, y, w, h)
        boxes_nms = np.hstack([boxes_640[:, :2], boxes_640[:, 2:] - boxes_640[:, :2]]
        )
        indices = cv2.dnn.NMSBoxes(
            boxes_nms.tolist(), scores.tolist(),
            self.conf_thres, self.iou_thres
        )
        if len(indices) == 0:
            return (np.empty((0, 4), dtype=np.int32),
                    np.empty(0), np.empty(0), [])
        indices = indices.flatten()

        boxes_640 = boxes_640[indices]
        scores = scores[indices]
        class_ids = class_ids[indices]
        mask_coeffs = mask_coeffs[indices]

        # Decode masks
        top_pad, left_pad, scale, orig_w, orig_h = pad_info
        masks_original = []
        prototypes_flat = prototypes.reshape(32, -1)  # (32, 25600)

        for i in range(len(boxes_640)):
            coeff = mask_coeffs[i]  # (32,)
            mask_raw = coeff @ prototypes_flat  # (25600,)
            mask_raw = 1.0 / (1.0 + np.exp(-mask_raw))  # sigmoid
            mask_raw = mask_raw.reshape(160, 160)
            mask_raw = (mask_raw > 0.5).astype(np.uint8) * 255

            # Resize mask to model input size
            mask_640 = cv2.resize(
                mask_raw,
                (self.input_width, self.input_height),
                interpolation=cv2.INTER_NEAREST
            )

            # Crop to bounding box (optional)
            x1b, y1b, x2b, y2b = boxes_640[i].astype(np.int32)
            mask_cropped = np.zeros(
                (self.input_height, self.input_width), dtype=np.uint8
            )
            mask_cropped[y1b:y2b, x1b:x2b] = mask_640[y1b:y2b, x1b:x2b]

            # Remove padding and scale back to original image size
            padded_h = int(orig_h * scale)
            padded_w = int(orig_w * scale)
            mask_no_pad = mask_cropped[
                top_pad:top_pad + padded_h,
                left_pad:left_pad + padded_w
            ]
            mask_original = cv2.resize(
                mask_no_pad, (orig_w, orig_h),
                interpolation=cv2.INTER_NEAREST
            )

            masks_original.append(mask_original)

        # Convert bounding boxes from model space to original image coordinates
        boxes_original = boxes_640.copy()
        boxes_original[:, [0, 2]] = (boxes_original[:, [0, 2]] - left_pad) / scale
        boxes_original[:, [1, 3]] = (boxes_original[:, [1, 3]] - top_pad) / scale
        boxes_original = np.clip(
            boxes_original,
            0, [orig_w, orig_h, orig_w, orig_h]
        ).astype(np.int32)

        return boxes_original, scores, class_ids, masks_original

    def draw_segmentation(self, img, boxes, scores, class_ids, masks):
        """Draw bounding boxes, labels, and semi-transparent masks."""
        result = img.copy()
        overlay = np.zeros_like(img, dtype=np.uint8)

        np.random.seed(42)
        colors = np.random.randint(0, 255, size=(80, 3), dtype=np.uint8)

        # Draw masks on overlay
        for mask, cls_id in zip(masks, class_ids):
            color = colors[cls_id % 80].tolist()
            mask_bool = mask > 0
            overlay[mask_bool] = color

        # Blend overlay with original image
        result = cv2.addWeighted(result, 0.6, overlay, 0.4, 0)

        # Draw bounding boxes and labels on the blended image
        for box, score, cls_id in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = box
            color = colors[cls_id % 80].tolist()
            cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
            class_name = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else f"Class {cls_id}"
            label = f"{class_name}: {score:.2f}"
            cv2.putText(
                result, label, (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
            )

        return result

    def generate_label_image(self, masks, class_ids, shape):
        """Генерирует int32 label image в пространстве ADE20K."""
        label_img = np.zeros(shape[:2], dtype=np.int32)  # 0 = background

        for mask,cls_id in zip(masks, class_ids):
            coco_id = int(cls_id)
            ade_id = COCO_TO_ADE20K.get(coco_id, 0)
            if ade_id == 0:
                continue
            label_img[mask > 0] = ade_id

        return label_img

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        input_tensor, pad_info = self.preprocess(cv_image)

        try:
            outputs = self.rknn.inference(inputs=[input_tensor])
        except Exception as e:
            self.get_logger().error(f"RKNN inference failed: {e}")
            return

        boxes, scores, class_ids, masks = self.postprocess(outputs, pad_info)

        # Визуализация
        result_img = self.draw_segmentation(cv_image, boxes, scores, class_ids, masks)
        result_msg = self.bridge.cv2_to_imgmsg(result_img, encoding='bgr8')
        result_msg.header = msg.header
        self.publisher.publish(result_msg)

        # Label image для Hydra (encoding=32SC1)
        label_img = self.generate_label_image(masks, class_ids, cv_image.shape)
        label_msg = self.bridge.cv2_to_imgmsg(label_img, encoding='32SC1')
        label_msg.header = msg.header
        self.label_publisher.publish(label_msg)

        self.get_logger().debug(f"Published result with {len(boxes)} detections")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
