#!/usr/bin/env python3
"""
frame_speed_divider_node
========================
Extracts the three horizontal column strips used by the speed pipeline
(Section 3 and Section 7 of the technical report).

Frame column layout (1280 px wide):
    ┌──────────┬────────────────────────┬──────────┐
    │  Conv1   │        Vision          │  Conv2   │
    │ cols 0–  │    cols 257 – 1024     │ cols 1025│
    │   256    │     (768 px wide)      │  – 1280  │
    └──────────┴────────────────────────┴──────────┘

Publishes
---------
/speed_frame/conv1    sensor_msgs/Image   — left encoder strip
/speed_frame/vision   sensor_msgs/Image   — centre zone (centroid tracking)
/speed_frame/conv2    sensor_msgs/Image   — right encoder strip
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FrameSpeedDividerNode(Node):
    """Publishes three column-strip sub-frames for speed computation."""

    def __init__(self):
        super().__init__('frame_speed_divider_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('conv1_col_start',  0)
        self.declare_parameter('conv1_col_end',    256)
        self.declare_parameter('vision_col_start', 257)
        self.declare_parameter('vision_col_end',   1024)
        self.declare_parameter('conv2_col_start',  1025)
        self.declare_parameter('conv2_col_end',    1280)

        self._strips = {
            'conv1':  (self.get_parameter('conv1_col_start').value,
                       self.get_parameter('conv1_col_end').value),
            'vision': (self.get_parameter('vision_col_start').value,
                       self.get_parameter('vision_col_end').value),
            'conv2':  (self.get_parameter('conv2_col_start').value,
                       self.get_parameter('conv2_col_end').value),
        }

        # ── Bridge & QoS ──────────────────────────────────────────────────
        self._bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=1)

        # ── Publishers ────────────────────────────────────────────────────
        self._pubs = {
            name: self.create_publisher(Image, f'/speed_frame/{name}', qos)
            for name in self._strips
        }

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(Image, '/raw_frame', self._callback, qos)
        self.get_logger().info('frame_speed_divider_node ready.')

    def _callback(self, msg: Image) -> None:
        full = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        stamp = msg.header.stamp

        for name, (c_start, c_end) in self._strips.items():
            strip = full[:, c_start:c_end]          # full-height column slice
            out = self._bridge.cv2_to_imgmsg(strip, encoding='bgr8')
            out.header.stamp    = stamp
            out.header.frame_id = f'speed_{name}'
            self._pubs[name].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FrameSpeedDividerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
