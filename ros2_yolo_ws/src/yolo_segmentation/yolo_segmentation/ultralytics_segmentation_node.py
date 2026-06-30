#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO


COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]


COCO_TO_ADE20K = {
    0: 12,    # person -> person
    13: 69,   # bench -> bench
    56: 19,   # chair -> chair
    57: 23,   # couch -> sofa
    58: 17,   # potted plant -> plant
    59: 7,    # bed -> bed
    60: 15,   # dining table -> table
    61: 65,   # toilet -> toilet
    39: 98,   # bottle -> bottle
    40: 147,  # wine glass -> drinking glass
    41: 147,  # cup -> drinking glass
    46: 120, 47: 120, 48: 120, 49: 120, 50: 120,
    51: 120, 52: 120, 53: 120, 54: 120, 55: 120,
    62: 89,   # tv -> television
    63: 74,   # laptop -> computer
    68: 124,  # microwave -> microwave
    69: 118,  # oven -> oven
    71: 47,   # sink -> sink
    72: 50,   # refrigerator -> refrigerator
    73: 67,   # book -> book
    74: 148,  # clock -> clock
    75: 135,  # vase -> vase
}


ADE20K_COLORS = {
    3: (255, 0, 0),  # floor
}


class UltralyticsSegmentationNode(Node):
    def __init__(self):
        super().__init__("ultralytics_segmentation_node")

        self.declare_parameter("model_path", "")
        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter(
            "depth_info_topic",
            "/camera/camera/aligned_depth_to_color/camera_info",
        )
        self.declare_parameter("result_topic", "/yolo_segmentation/result")
        self.declare_parameter("label_topic", "/yolo_segmentation/labels")
        self.declare_parameter("confidence_threshold", 0.2)
        self.declare_parameter("iou_threshold", 0.35)
        self.declare_parameter("image_size", 640)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("use_depth_floor", True)
        self.declare_parameter("h_floor_m", 0.23)
        self.declare_parameter("h_tolerance", 0.15)

        model_path = self.get_parameter("model_path").value
        if not model_path:
            model_path = self._default_model_path()

        self.input_topic = self.get_parameter("input_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.depth_info_topic = self.get_parameter("depth_info_topic").value
        self.result_topic = self.get_parameter("result_topic").value
        self.label_topic = self.get_parameter("label_topic").value
        self.conf_thres = self.get_parameter("confidence_threshold").value
        self.iou_thres = self.get_parameter("iou_threshold").value
        self.image_size = self.get_parameter("image_size").value
        self.device = self.get_parameter("device").value
        self.use_depth_floor = self.get_parameter("use_depth_floor").value
        self.h_floor = self.get_parameter("h_floor_m").value
        self.h_tol = self.get_parameter("h_tolerance").value

        self.bridge = CvBridge()
        self.latest_depth = None
        self.cam_fy = None
        self.cam_cy = None

        self.get_logger().info(f"Loading Ultralytics model: {model_path}")
        self.model = YOLO(model_path)

        self.create_subscription(Image, self.input_topic, self.image_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)
        self.depth_info_sub = self.create_subscription(
            CameraInfo,
            self.depth_info_topic,
            self.camera_info_callback,
            10,
        )
        self.result_pub = self.create_publisher(Image, self.result_topic, 10)
        self.label_pub = self.create_publisher(Image, self.label_topic, 10)

        self.get_logger().info(
            "Node started.\n"
            f"  RGB    : {self.input_topic}\n"
            f"  Depth  : {self.depth_topic}\n"
            f"  Labels : {self.label_topic}\n"
            f"  Result : {self.result_topic}\n"
            f"  Device : {self.device}"
        )

    @staticmethod
    def _default_model_path():
        package_share = Path(get_package_share_directory("yolo_segmentation"))
        return str(package_share / "models" / "yolov8n-seg.pt")

    def camera_info_callback(self, msg):
        if self.cam_fy is not None:
            return

        self.cam_fy = msg.k[4]
        self.cam_cy = msg.k[5]
        self.get_logger().info(
            f"Camera intrinsics received: fy={self.cam_fy:.2f}, cy={self.cam_cy:.2f}"
        )
        self.destroy_subscription(self.depth_info_sub)

    def depth_callback(self, msg):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"Depth conversion failed: {exc}")
            return

        if depth_raw.dtype == np.uint16:
            self.latest_depth = depth_raw.astype(np.float32) / 1000.0
        else:
            self.latest_depth = depth_raw.astype(np.float32)

    def classify_background(self, shape):
        h, w = shape[:2]
        label_bg = np.zeros((h, w), dtype=np.int32)
        if not self.use_depth_floor:
            return label_bg
        if self.latest_depth is None or self.cam_fy is None:
            return label_bg

        depth = self.latest_depth
        if depth.shape[:2] != (h, w):
            depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_NEAREST)

        rows = np.tile(np.arange(h, dtype=np.float32).reshape(h, 1), (1, w))
        valid = depth > 0.1
        y3d = np.where(valid, (rows - self.cam_cy) * depth / self.cam_fy, np.nan)
        floor_mask = valid & (y3d >= self.h_floor) & (y3d <= self.h_floor + self.h_tol)
        label_bg[floor_mask] = 3
        return label_bg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")
            return

        try:
            predictions = self.model.predict(
                source=cv_image,
                conf=self.conf_thres,
                iou=self.iou_thres,
                imgsz=self.image_size,
                device=self.device,
                retina_masks=True,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(f"Ultralytics inference failed: {exc}")
            return

        boxes, scores, class_ids, masks = self.parse_predictions(predictions, cv_image.shape)
        label_img = self.generate_label_image(masks, class_ids, cv_image.shape)

        label_msg = self.bridge.cv2_to_imgmsg(label_img, encoding="32SC1")
        label_msg.header = msg.header
        self.label_pub.publish(label_msg)

        result_img = self.draw_segmentation(cv_image, boxes, scores, class_ids, masks, label_img)
        result_msg = self.bridge.cv2_to_imgmsg(result_img, encoding="bgr8")
        result_msg.header = msg.header
        self.result_pub.publish(result_msg)

        self.get_logger().debug(
            f"Published {len(boxes)} detections; "
            f"depth_ok={self.latest_depth is not None}; "
            f"intrinsics_ok={self.cam_fy is not None}"
        )

    def parse_predictions(self, predictions, image_shape):
        if not predictions:
            return [], [], [], []

        result = predictions[0]
        if result.boxes is None or len(result.boxes) == 0:
            return [], [], [], []

        boxes = result.boxes.xyxy.cpu().numpy().astype(np.int32)
        scores = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy().astype(np.int32)

        h, w = image_shape[:2]
        if result.masks is None:
            masks = [self.box_to_mask(box, h, w) for box in boxes]
            return boxes, scores, class_ids, masks

        mask_data = result.masks.data.cpu().numpy()
        masks = []
        for mask in mask_data:
            if mask.shape[:2] != (h, w):
                mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
            masks.append((mask > 0.5).astype(np.uint8) * 255)

        return boxes, scores, class_ids, masks

    @staticmethod
    def box_to_mask(box, h, w):
        x1, y1, x2, y2 = np.clip(box, 0, [w, h, w, h]).astype(np.int32)
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[y1:y2, x1:x2] = 255
        return mask

    def generate_label_image(self, masks, class_ids, shape):
        label_img = self.classify_background(shape)
        for mask, cls_id in zip(masks, class_ids):
            ade_id = COCO_TO_ADE20K.get(int(cls_id), 0)
            if ade_id:
                label_img[mask > 0] = ade_id
        return label_img

    def draw_segmentation(self, img, boxes, scores, class_ids, masks, label_img):
        result = img.copy()
        overlay = np.zeros_like(img, dtype=np.uint8)

        for ade_id, color in ADE20K_COLORS.items():
            overlay[label_img == ade_id] = color

        rng = np.random.default_rng(42)
        obj_colors = rng.integers(0, 255, size=(80, 3), dtype=np.uint8)
        for mask, cls_id in zip(masks, class_ids):
            overlay[mask > 0] = obj_colors[int(cls_id) % 80].tolist()

        blended = cv2.addWeighted(img, 0.55, overlay, 0.45, 0)
        mask_exists = np.any(overlay > 0, axis=2, keepdims=True)
        result = np.where(mask_exists, blended, img)

        for box, score, cls_id in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = box
            color = obj_colors[int(cls_id) % 80].tolist()
            cv2.rectangle(result, (x1, y1), (x2, y2), color, 2)
            name = COCO_CLASSES[int(cls_id)] if cls_id < len(COCO_CLASSES) else f"cls{cls_id}"
            cv2.putText(
                result,
                f"{name}:{score:.2f}",
                (x1, max(y1 - 5, 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )

        return result


def main(args=None):
    rclpy.init(args=args)
    node = UltralyticsSegmentationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
