#!/usr/bin/env python3
"""
volume_estimation_node
=======================
Estimates the 3D volume of each pear using the depth sub-frame.

Method
------
For each valid pixel inside the pear silhouette, the depth value (mm)
combined with the pixel-to-cm scale gives a column height.  Summing all
column volumes (pixel_area × depth) approximates total pear volume.

Subscribes
----------
/depth_frame/{zone_id}   sensor_msgs/Image   (16UC1, mm)
/{zone_id}/area          std_msgs/Float32    (silhouette area in px²)

Publishes
---------
/{zone_id}/volume        std_msgs/Float32    (volume in cm³)
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

_ZONES = ["A1", "A2", "A3", "B1", "B2", "B3"]


class VolumeEstimationNode(Node):
    """Estimates pear volume from depth data for all 6 zones."""

    def __init__(self):
        super().__init__('volume_estimation_node')

        self.declare_parameter('px_to_cm', 0.05)
        self._px_to_cm: float = self.get_parameter('px_to_cm').value

        self._bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=1)

        # Cache the latest area per zone
        self._areas: dict[str, float] = {z: 0.0 for z in _ZONES}
        self._pubs:  dict[str, rclpy.publisher.Publisher] = {}

        for zone in _ZONES:
            self._pubs[zone] = self.create_publisher(
                Float32, f'/{zone}/volume', qos
            )
            # Subscribe to zone depth frame
            self.create_subscription(
                Image,
                f'/depth_frame/{zone}',
                self._make_depth_cb(zone),
                qos
            )
            # Subscribe to zone area (cached for use in depth callback)
            self.create_subscription(
                Float32,
                f'/{zone}/area',
                self._make_area_cb(zone),
                qos
            )

        self.get_logger().info('volume_estimation_node ready.')

    def _make_area_cb(self, zone: str):
        def _cb(msg: Float32):
            self._areas[zone] = msg.data
        return _cb

    def _make_depth_cb(self, zone: str):
        def _cb(msg: Image):
            if self._areas[zone] < 1.0:
                return     # No pear detected in this zone

            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            # Valid depth pixels: non-zero and within 0–500 mm (close range)
            valid = depth[(depth > 0) & (depth < 500)]
            if len(valid) == 0:
                return

            # Simple hemisphere approximation:
            # mean_depth_cm × pixel_area_cm² → volume in cm³
            mean_depth_mm = float(np.mean(valid))
            mean_depth_cm = mean_depth_mm / 10.0

            area_cm2 = self._areas[zone] * (self._px_to_cm ** 2)
            # Hemisphere: V ≈ (2/3)π r³  but we use column-sum approximation
            # volume ≈ area_cm2 × (mean_depth_cm / 2) as a simplified model
            volume_cm3 = area_cm2 * (mean_depth_cm / 2.0)

            out = Float32()
            out.data = float(volume_cm3)
            self._pubs[zone].publish(out)
        return _cb


def main(args=None):
    rclpy.init(args=args)
    node = VolumeEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
