#!/usr/bin/env python3
"""
frame_capture_node
==================
Entry point of the entire SAAT pipeline.

Responsibilities
----------------
* Opens the Intel RealSense D455 at 1280×720 for both colour and depth streams.
* Aligns the depth frame to the colour frame (pixel-perfect overlay).
* Publishes:
    /raw_frame        sensor_msgs/Image   (BGR8, colour)
    /raw_depth        sensor_msgs/Image   (16UC1, depth in mm)

All downstream divider nodes subscribe to these two topics.

Critical timing constraint (Section 17)
----------------------------------------
The camera runs at 30 fps inside Docker, giving a 33 ms inter-frame window.
The 1-second action cycle means we have ~30 frames to complete vision + PID +
actuation.  This node must never block longer than one frame period.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import pyrealsense2 as rs


class FrameCaptureNode(Node):
    """Captures aligned colour + depth frames from the RealSense D455."""

    def __init__(self):
        super().__init__('frame_capture_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('frame_width',        1280)
        self.declare_parameter('frame_height',       720)
        self.declare_parameter('color_fps',          30)
        self.declare_parameter('depth_fps',          30)
        self.declare_parameter('publish_topic',      '/raw_frame')
        self.declare_parameter('publish_depth_topic','/raw_depth')

        w   = self.get_parameter('frame_width').value
        h   = self.get_parameter('frame_height').value
        cfps = self.get_parameter('color_fps').value
        dfps = self.get_parameter('depth_fps').value
        col_topic   = self.get_parameter('publish_topic').value
        depth_topic = self.get_parameter('publish_depth_topic').value

        # ── Publishers ────────────────────────────────────────────────────
        # QoS depth=1: we only care about the latest frame, never queue up.
        qos = rclpy.qos.QoSProfile(depth=1)
        self._pub_color = self.create_publisher(Image, col_topic,   qos)
        self._pub_depth = self.create_publisher(Image, depth_topic, qos)
        self._bridge    = CvBridge()

        # ── RealSense pipeline ────────────────────────────────────────────
        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, cfps)
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16,  dfps)

        try:
            self._profile = self._pipeline.start(cfg)
            self.get_logger().info(
                f'RealSense D455 started: {w}×{h} colour@{cfps}fps depth@{dfps}fps'
            )
        except Exception as exc:
            self.get_logger().fatal(f'Cannot open RealSense camera: {exc}')
            raise

        # Align depth to colour coordinate space
        self._align = rs.align(rs.stream.color)

        # ── Timer: publish at colour FPS ──────────────────────────────────
        self._timer = self.create_timer(1.0 / cfps, self._capture_callback)

    # ── Callback ──────────────────────────────────────────────────────────
    def _capture_callback(self) -> None:
        """Grab one aligned frame pair and publish both images."""
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=100)
        except RuntimeError:
            self.get_logger().warn('Frame timeout — skipping cycle')
            return

        aligned   = self._align.process(frames)
        color_frm = aligned.get_color_frame()
        depth_frm = aligned.get_depth_frame()

        if not color_frm or not depth_frm:
            return

        # numpy arrays
        color_img = np.asanyarray(color_frm.get_data())   # (H, W, 3) uint8
        depth_img = np.asanyarray(depth_frm.get_data())   # (H, W)    uint16 mm

        # ROS stamp (shared between both messages for synchronisation)
        stamp = self.get_clock().now().to_msg()

        # Publish colour
        color_msg = self._bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
        color_msg.header.stamp    = stamp
        color_msg.header.frame_id = 'camera_color_optical_frame'
        self._pub_color.publish(color_msg)

        # Publish depth (16-bit unsigned, each value = mm)
        depth_msg = self._bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
        depth_msg.header.stamp    = stamp
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self._pub_depth.publish(depth_msg)

    # ── Lifecycle ─────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('Stopping RealSense pipeline…')
        self._pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrameCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
