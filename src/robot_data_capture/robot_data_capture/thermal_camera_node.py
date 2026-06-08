#!/usr/bin/env python3
"""Publish a V4L2 thermal camera as sensor_msgs/Image."""

import time
import subprocess

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image


class ThermalCameraNode(Node):
    """Capture MJPG frames from a V4L2 thermal camera at a fixed rate."""

    def __init__(self):
        super().__init__("thermal_camera_node")
        self.declare_parameter("device", "")
        self.declare_parameter("device_name", "USB2.0 PC CAMERA")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("frame_id", "thermal_camera")
        self.declare_parameter("image_topic", "/thermal/image_raw")
        self.declare_parameter("camera_info_topic", "/thermal/camera_info")
        self.declare_parameter("encoding", "bgr8")

        self._device = self.get_parameter("device").value
        self._device_name = self.get_parameter("device_name").value
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._fps = float(self.get_parameter("fps").value)
        self._frame_id = self.get_parameter("frame_id").value
        self._encoding = self.get_parameter("encoding").value

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._image_pub = self.create_publisher(
            Image, self.get_parameter("image_topic").value, qos
        )
        self._info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("camera_info_topic").value, qos
        )
        self._bridge = CvBridge()
        self._device = self._resolve_device(self._device, self._device_name)
        self._cap = self._open_capture()
        self._timer = self.create_timer(1.0 / self._fps, self._publish_frame)
        self.get_logger().info(
            f"Thermal capture {self._device}: {self._width}x{self._height}@{self._fps:g} MJPG"
        )

    def _device_supports_format(self, path):
        try:
            result = subprocess.run(
                ["v4l2-ctl", "-d", path, "--list-formats-ext"],
                check=True,
                capture_output=True,
                text=True,
            )
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

        output = result.stdout
        return (
            "'MJPG'" in output
            and f"Size: Discrete {self._width}x{self._height}" in output
            and f"({self._fps:.3f} fps)" in output
        )

    def _resolve_device(self, device, device_name):
        if device:
            return device

        try:
            result = subprocess.run(
                ["/usr/bin/v4l2-ctl", "--list-devices"],
                check=False,
                capture_output=True,
                text=True,
            )

            self.get_logger().info(f"v4l2-ctl returncode: {result.returncode}")
            self.get_logger().info(f"v4l2-ctl stdout:\n{result.stdout}")
            self.get_logger().info(f"v4l2-ctl stderr:\n{result.stderr}")

        except subprocess.CalledProcessError as exc:
            raise RuntimeError(
                "thermal device is not set and v4l2-ctl --list-devices failed\n"
                f"returncode: {exc.returncode}\n"
                f"stdout:\n{exc.stdout}\n"
                f"stderr:\n{exc.stderr}\n"
            ) from exc

        except FileNotFoundError as exc:
            raise RuntimeError(
                "thermal device is not set and /usr/bin/v4l2-ctl was not found. "
                "Install it with: sudo apt install v4l-utils"
            ) from exc
            
        current_name = None
        candidates = []
        for raw_line in result.stdout.splitlines():
            line = raw_line.rstrip()
            if not line:
                current_name = None
                continue
            if not raw_line.startswith(("\t", " ")):
                current_name = line.split(" (", 1)[0]
                continue
            path = line.strip()
            if current_name == device_name and path.startswith("/dev/video"):
                candidates.append(path)

        for path in candidates:
            if self._device_supports_format(path):
                self.get_logger().info(
                    f"Resolved thermal camera '{device_name}' to {path}"
                )
                return path

        if candidates:
            self.get_logger().warning(
                f"Found {device_name} candidates {candidates}, but none exposes "
                f"MJPG {self._width}x{self._height}@{self._fps:g}; trying {candidates[0]}"
            )
            return candidates[0]

        raise RuntimeError(
            f"Failed to find thermal camera '{device_name}' in v4l2-ctl --list-devices"
        )

    def _open_capture(self):
        cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open thermal camera {self._device}")

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _publish_frame(self):
        ok, frame = self._cap.read()
        stamp = self.get_clock().now().to_msg()
        if not ok or frame is None:
            self.get_logger().warning("Failed to read thermal frame")
            time.sleep(0.01)
            return

        if self._encoding == "mono8" and len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        image_msg = self._bridge.cv2_to_imgmsg(frame, encoding=self._encoding)
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self._frame_id

        info_msg = CameraInfo()
        info_msg.header = image_msg.header
        info_msg.width = image_msg.width
        info_msg.height = image_msg.height

        self._image_pub.publish(image_msg)
        self._info_pub.publish(info_msg)

    def destroy_node(self):
        if hasattr(self, "_cap"):
            self._cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = ThermalCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
