#!/usr/bin/env python3
"""
volume_divider_node
===================
Routes the aligned depth frame into 6 zone sub-frames (same ROI grid as
frame_divider_node) for volume and mass estimation.

Publishes
---------
/depth_frame/A1  …  /depth_frame/B3    sensor_msgs/Image  (16UC1, depth in mm)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

_DEFAULT_ROIS = {
    "A1": (0,   0,   427, 360),
    "A2": (427, 0,   854, 360),
    "A3": (854, 0,  1280, 360),
    "B1": (0,   360, 427, 720),
    "B2": (427, 360, 854, 720),
    "B3": (854, 360, 1280, 720),
}
_DEFAULT_ORDER = ["A1", "A2", "A3", "B1", "B2", "B3"]


class VolumeDividerNode(Node):
    """Splits depth frames into 6 zone-aligned depth sub-frames."""

    def __init__(self):
        super().__init__('volume_divider_node')

        self.declare_parameter('roi_order', _DEFAULT_ORDER)
        roi_order = self.get_parameter('roi_order').value

        self._rois: dict[str, tuple] = {}
        for zone in roi_order:
            self.declare_parameter(f'rois.{zone}', list(_DEFAULT_ROIS[zone]))
            raw = self.get_parameter(f'rois.{zone}').value
            self._rois[zone] = tuple(int(v) for v in raw)

        self._bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=1)

        self._pubs = {
            zone: self.create_publisher(Image, f'/depth_frame/{zone}', qos)
            for zone in roi_order
        }

        self.create_subscription(Image, '/raw_depth', self._callback, qos)
        self.get_logger().info('volume_divider_node ready.')

    def _callback(self, msg: Image) -> None:
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        stamp = msg.header.stamp

        for zone, (xs, ys, xe, ye) in self._rois.items():
            sub = depth[ys:ye, xs:xe]
            out = self._bridge.cv2_to_imgmsg(sub, encoding='16UC1')
            out.header.stamp    = stamp
            out.header.frame_id = f'depth_{zone}'
            self._pubs[zone].publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VolumeDividerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
