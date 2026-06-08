#!/usr/bin/env python3
"""Publish microphone PCM frames using arecord."""

import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from robot_data_capture.msg import AudioFrame


class MicrophoneNode(Node):
    """Stream audio chunks from ALSA arecord into ROS messages."""

    def __init__(self):
        super().__init__("microphone_node")
        self.declare_parameter("device", "default")
        self.declare_parameter("topic", "/microphone/audio")
        self.declare_parameter("frame_id", "microphone")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("sample_format", "S16_LE")
        self.declare_parameter("encoding", "pcm_s16le")
        self.declare_parameter("chunk_ms", 100)

        self._device = self.get_parameter("device").value
        self._topic = self.get_parameter("topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._sample_rate = int(self.get_parameter("sample_rate").value)
        self._channels = int(self.get_parameter("channels").value)
        self._sample_format = self.get_parameter("sample_format").value
        self._encoding = self.get_parameter("encoding").value
        self._chunk_ms = int(self.get_parameter("chunk_ms").value)
        self._bytes_per_sample = 2
        self._chunk_bytes = max(
            1,
            int(
                self._sample_rate
                * self._channels
                * self._bytes_per_sample
                * self._chunk_ms
                / 1000
            ),
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(AudioFrame, self._topic, qos)
        self._stop_event = threading.Event()
        self._process = self._start_arecord()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"Microphone capture {self._device}: {self._sample_rate} Hz, "
            f"{self._channels} ch, {self._sample_format}, chunks {self._chunk_ms} ms"
        )

    def _start_arecord(self):
        cmd = [
            "arecord",
            "-q",
            "-D",
            self._device,
            "-f",
            self._sample_format,
            "-r",
            str(self._sample_rate),
            "-c",
            str(self._channels),
            "-t",
            "raw",
        ]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def _read_loop(self):
        assert self._process.stdout is not None
        while rclpy.ok() and not self._stop_event.is_set():
            data = self._process.stdout.read(self._chunk_bytes)
            if not data:
                if self._process.poll() is not None:
                    stderr = b""
                    if self._process.stderr is not None:
                        stderr = self._process.stderr.read()
                    self.get_logger().error(
                        "arecord stopped: " + stderr.decode(errors="replace").strip()
                    )
                    return
                continue

            msg = AudioFrame()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            msg.sample_rate = self._sample_rate
            msg.channels = self._channels
            msg.encoding = self._encoding
            msg.data = list(data)
            self._pub.publish(msg)

    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, "_process") and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
        super().destroy_node()


def main():
    rclpy.init()
    node = MicrophoneNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
