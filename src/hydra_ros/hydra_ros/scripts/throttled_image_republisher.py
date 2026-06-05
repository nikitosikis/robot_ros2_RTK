#!/usr/bin/env python3
"""Republish the latest image at a fixed low rate for RViz."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import Image


class ThrottledImageRepublisher(Node):
    """Keep graph inputs fast while giving RViz a low-bandwidth image topic."""

    def __init__(self):
        super().__init__("throttled_image_republisher")
        self.declare_parameter("input_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("output_topic", "/rviz/camera/color/image_raw")
        self.declare_parameter("rate_hz", 1.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive")

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._latest_msg = None
        self._sub = self.create_subscription(Image, input_topic, self._callback, sub_qos)
        self._pub = self.create_publisher(Image, output_topic, pub_qos)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_latest)

        self.get_logger().info(
            f"Republishing {input_topic} to {output_topic} at {rate_hz:.2f} Hz"
        )

    def _callback(self, msg):
        self._latest_msg = msg

    def _publish_latest(self):
        if self._latest_msg is not None:
            self._pub.publish(self._latest_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrottledImageRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
