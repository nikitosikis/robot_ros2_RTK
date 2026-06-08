#!/usr/bin/env python3
"""Approximate-sync RealSense RGB and thermal frames for bag recording."""

import copy

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class ImagePairSyncNode(Node):
    """Republish camera/thermal image pairs with one shared header stamp."""

    def __init__(self):
        super().__init__("image_pair_sync_node")
        self.declare_parameter("camera_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("thermal_topic", "/thermal/image_raw")
        self.declare_parameter("synced_camera_topic", "/data_capture/camera/image_raw")
        self.declare_parameter("synced_thermal_topic", "/data_capture/thermal/image_raw")
        self.declare_parameter("queue_size", 15)
        self.declare_parameter("slop_sec", 0.08)

        queue_size = int(self.get_parameter("queue_size").value)
        slop_sec = float(self.get_parameter("slop_sec").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        camera_topic = self.get_parameter("camera_topic").value
        thermal_topic = self.get_parameter("thermal_topic").value
        self._camera_pub = self.create_publisher(
            Image, self.get_parameter("synced_camera_topic").value, qos
        )
        self._thermal_pub = self.create_publisher(
            Image, self.get_parameter("synced_thermal_topic").value, qos
        )

        self._camera_sub = message_filters.Subscriber(
            self, Image, camera_topic, qos_profile=qos
        )
        self._thermal_sub = message_filters.Subscriber(
            self, Image, thermal_topic, qos_profile=qos
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._camera_sub, self._thermal_sub],
            queue_size=queue_size,
            slop=slop_sec,
            allow_headerless=False,
        )
        self._sync.registerCallback(self._callback)
        self.get_logger().info(
            f"Syncing {camera_topic} and {thermal_topic} with slop {slop_sec:.3f}s"
        )

    def _callback(self, camera_msg, thermal_msg):
        stamp = self.get_clock().now().to_msg()
        synced_camera = copy.deepcopy(camera_msg)
        synced_thermal = copy.deepcopy(thermal_msg)
        synced_camera.header.stamp = stamp
        synced_thermal.header.stamp = stamp
        self._camera_pub.publish(synced_camera)
        self._thermal_pub.publish(synced_thermal)


def main():
    rclpy.init()
    node = ImagePairSyncNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
