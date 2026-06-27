#!/usr/bin/env python3
"""
area_node  (A1_area_node … B3_area_node)
==========================================
Computes the 2D surface area (in pixels²) of the pear silhouette from
the colour zone sub-frame.  Reuses the same HSV+LAB segmentation masks
as the vision node but only needs the final contour area — no Otsu step.

Subscribes
----------
/zone_frame/{zone_id}    sensor_msgs/Image        (BGR8 colour sub-frame)
/vision_params           std_msgs/String          (JSON config, latched)

Publishes
---------
/{zone_id}/area          std_msgs/Float32         (pear area in pixels²)
"""

import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge


class AreaNode(Node):
    """Measures pear silhouette area for one belt zone."""

    def __init__(self):
        super().__init__('area_node')

        self.declare_parameter('zone_id', 'A1')
        self._zone = self.get_parameter('zone_id').value
        self._params: dict | None = None
        self._bridge = CvBridge()

        std_qos = rclpy.qos.QoSProfile(depth=1)
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._pub = self.create_publisher(Float32, f'/{self._zone}/area', std_qos)

        self.create_subscription(String, '/vision_params', self._params_cb, latch_qos)
        self.create_subscription(Image, f'/zone_frame/{self._zone}', self._frame_cb, std_qos)

        self.get_logger().info(f'[{self._zone}] area_node ready.')

    def _params_cb(self, msg: String) -> None:
        self._params = json.loads(msg.data)

    def _frame_cb(self, msg: Image) -> None:
        if self._params is None:
            return

        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        p   = self._params

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

        hsv_mask = cv2.inRange(
            hsv,
            np.array(p['hsv_lower'], dtype=np.uint8),
            np.array(p['hsv_upper'], dtype=np.uint8)
        )
        lab_mask = cv2.inRange(
            lab,
            np.array(p['lab_lower'], dtype=np.uint8),
            np.array(p['lab_upper'], dtype=np.uint8)
        )
        mask = cv2.bitwise_and(hsv_mask, lab_mask)

        k = p['morph_kernel_size']
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        cnts_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours  = cnts_info[0] if len(cnts_info) == 2 else cnts_info[1]

        area = 0.0
        if contours:
            main = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(main))
            if area < p['min_pear_area_px']:
                area = 0.0

        out = Float32()
        out.data = area
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AreaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
