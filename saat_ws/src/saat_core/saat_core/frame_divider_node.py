#!/usr/bin/env python3
"""
frame_divider_node
==================
Slices the full 1280×720 colour frame into 6 zone sub-frames (A1–B3)
and publishes each to its dedicated topic.

Belt zone layout (as viewed from camera above):
    ┌────────┬────────┬────────┐
    │   A1   │   A2   │   A3   │  ← top row    (y: 0   – 359)
    ├────────┼────────┼────────┤
    │   B1   │   B2   │   B3   │  ← bottom row (y: 360 – 719)
    └────────┴────────┴────────┘
     col: 0–426  427–853  854–1279

Publishes
---------
/zone_frame/A1   …   /zone_frame/B3    sensor_msgs/Image  (BGR8)

Timing note
-----------
All 6 crops are numpy array slices (zero-copy views) — no pixel copying occurs.
The only allocation is in cv2_to_imgmsg() for the message buffer.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


# ── ROI definitions ────────────────────────────────────────────────────────
# (x_start, y_start, x_end, y_end) in pixels  — matches saat_params.yaml
# Loaded dynamically from the parameter server; these are only fallback defaults.
_DEFAULT_ROIS = {
    "A1": (0,   0,   427, 360),
    "A2": (427, 0,   854, 360),
    "A3": (854, 0,  1280, 360),
    "B1": (0,   360, 427, 720),
    "B2": (427, 360, 854, 720),
    "B3": (854, 360, 1280, 720),
}
_DEFAULT_ORDER = ["A1", "A2", "A3", "B1", "B2", "B3"]


class FrameDividerNode(Node):
    """Divides the full colour frame into 6 zone sub-frames."""

    def __init__(self):
        super().__init__('frame_divider_node')

        # ── Parameters ────────────────────────────────────────────────────
        # ROIs are stored as flat lists in YAML; reconstruct tuples here.
        self.declare_parameter('roi_order', _DEFAULT_ORDER)
        roi_order: list = self.get_parameter('roi_order').value

        self._rois: dict[str, tuple] = {}
        for zone in roi_order:
            self.declare_parameter(f'rois.{zone}', list(_DEFAULT_ROIS[zone]))
            raw = self.get_parameter(f'rois.{zone}').value
            self._rois[zone] = tuple(int(v) for v in raw)   # (xs, ys, xe, ye)

        self.get_logger().info(f'Zone ROIs loaded: {self._rois}')

        # ── Bridge ────────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── Publishers — one per zone, QoS depth=1 ────────────────────────
        qos = rclpy.qos.QoSProfile(depth=1)
        self._pubs: dict[str, rclpy.publisher.Publisher] = {}
        for zone in roi_order:
            topic = f'/zone_frame/{zone}'
            self._pubs[zone] = self.create_publisher(Image, topic, qos)
            self.get_logger().info(f'Publishing zone {zone} → {topic}')

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(
            Image,
            '/raw_frame',
            self._frame_callback,
            qos
        )

    # ── Callback ──────────────────────────────────────────────────────────
    def _frame_callback(self, msg: Image) -> None:
        """
        Slice the full frame into zone sub-frames and publish each.
        Uses numpy array views for zero-copy slicing.
        """
        full_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        stamp = msg.header.stamp

        for zone, (xs, ys, xe, ye) in self._rois.items():
            # numpy slice — this is a *view*, not a copy
            sub = full_img[ys:ye, xs:xe]

            out_msg = self._bridge.cv2_to_imgmsg(sub, encoding='bgr8')
            out_msg.header.stamp    = stamp           # preserve original timestamp
            out_msg.header.frame_id = f'zone_{zone}'
            self._pubs[zone].publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrameDividerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
