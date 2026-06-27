#!/usr/bin/env python3
"""
vision_node  (A1_vision … B3_vision)
=====================================
One instance runs per belt zone.  The zone identity is set via the
`zone_id` ROS2 parameter (e.g. "A1").

9-Step Classical Vision Pipeline (Section 8 of technical report)
-----------------------------------------------------------------
Step 1 — CLAHE        : Adaptive histogram equalisation for uneven lighting.
Step 2 — Bilateral    : Edge-preserving noise reduction.
Step 3 — Colour space : BGR → HSV  and  BGR → LAB  in parallel.
Step 4 — Colour range : Mask pear body (HSV) and depth zone (depth mask).
Step 5 — Bitwise AND  : Combine HSV + LAB + depth masks → pear silhouette.
Step 6 — Morphology   : Opening (remove noise) + Closing (fill holes).
Step 7 — Otsu         : Adaptive threshold on pear-only grayscale pixels.
Step 8 — Contour      : Find infection regions, compute area / centroid / colour.
Step 9 — Output       : Publish InfectionResult to downstream nodes.

Subscribes
----------
/zone_frame/{zone_id}   sensor_msgs/Image   (BGR8 colour sub-frame)
/vision_params          std_msgs/String     (JSON config from vision_init_node)

Publishes
---------
/{zone_id}/detection    saat_interfaces/InfectionResult
"""

import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from saat_interfaces.msg import InfectionResult


class VisionNode(Node):
    """
    Runs the 9-step infection detection pipeline on one belt zone.
    Instantiate 6 times with different zone_id parameters.
    """

    def __init__(self):
        super().__init__('vision_node')

        # ── Zone identity ──────────────────────────────────────────────────
        self.declare_parameter('zone_id', 'A1')
        self._zone: str = self.get_parameter('zone_id').value

        # Rename the node to reflect its zone (cosmetic, aids debugging)
        # rclpy does not support renaming after init; use the launch file's
        # `name` argument instead (done in saat_launch.py).
        self.get_logger().info(f'Vision node starting for zone: {self._zone}')

        # ── Vision parameters (will be populated from /vision_params) ─────
        self._params: dict | None = None
        self._clahe: cv2.CLAHE | None = None

        # ── Bridge & QoS ──────────────────────────────────────────────────
        self._bridge = CvBridge()
        standard_qos = rclpy.qos.QoSProfile(depth=1)

        # ── Publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            InfectionResult,
            f'/{self._zone}/detection',
            standard_qos
        )

        # ── Latched subscriber for vision params ──────────────────────────
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            '/vision_params',
            self._params_callback,
            latch_qos
        )

        # ── Frame subscriber (only active after params are received) ───────
        self.create_subscription(
            Image,
            f'/zone_frame/{self._zone}',
            self._frame_callback,
            standard_qos
        )

        self.get_logger().info(
            f'[{self._zone}] Waiting for /vision_params before processing frames…'
        )

    # ── Parameter callback ────────────────────────────────────────────────
    def _params_callback(self, msg: String) -> None:
        """Deserialise vision parameters from JSON and build OpenCV objects."""
        self._params = json.loads(msg.data)

        tile = tuple(self._params['clahe_tile_size'])
        self._clahe = cv2.createCLAHE(
            clipLimit=self._params['clahe_clip_limit'],
            tileGridSize=tile
        )
        self.get_logger().info(
            f'[{self._zone}] Vision params received — pipeline active.'
        )

    # ── Main vision callback ──────────────────────────────────────────────
    def _frame_callback(self, msg: Image) -> None:
        """Execute the 9-step pipeline on one sub-frame."""
        if self._params is None:
            # Params not yet received from vision_init_node — skip frame.
            return

        color_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self._run_pipeline(color_img)

        out = InfectionResult()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = self._zone
        out.zone_id         = self._zone

        out.pear_detected      = result['pear_detected']
        out.is_infected        = result['is_infected']
        out.infection_ratio    = float(result['infection_ratio'])
        out.infection_area_px  = float(result['infection_area_px'])
        out.pear_area_px       = float(result['pear_area_px'])
        out.infection_x        = float(result['infection_x'])
        out.infection_y        = float(result['infection_y'])
        out.infection_r        = int(result['infection_r'])
        out.infection_g        = int(result['infection_g'])
        out.infection_b        = int(result['infection_b'])
        out.pear_centroid_x    = float(result['pear_cx'])
        out.pear_centroid_y    = float(result['pear_cy'])

        self._pub.publish(out)

    # ── 9-Step Pipeline ───────────────────────────────────────────────────
    def _run_pipeline(self, color_img: np.ndarray) -> dict:
        """
        Execute all 9 steps and return a result dictionary.
        Returns a 'no-pear' result dict if no valid contour is found.
        """
        p = self._params   # shorthand

        # ── Step 1: CLAHE — adaptive contrast enhancement ─────────────────
        lab_for_clahe = cv2.cvtColor(color_img, cv2.COLOR_BGR2LAB)
        l_ch, a_ch, b_ch = cv2.split(lab_for_clahe)
        l_ch = self._clahe.apply(l_ch)
        enhanced = cv2.merge([l_ch, a_ch, b_ch])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)

        # ── Step 2: Bilateral filter — edge-preserving noise reduction ─────
        filtered = cv2.bilateralFilter(
            enhanced,
            p['bilateral_d'],
            p['bilateral_sigma_color'],
            p['bilateral_sigma_space']
        )

        # ── Step 3: Colour space conversions ──────────────────────────────
        hsv = cv2.cvtColor(filtered, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(filtered, cv2.COLOR_BGR2LAB)

        # ── Step 4: Colour range masks ────────────────────────────────────
        hsv_lower = np.array(p['hsv_lower'], dtype=np.uint8)
        hsv_upper = np.array(p['hsv_upper'], dtype=np.uint8)
        lab_lower = np.array(p['lab_lower'], dtype=np.uint8)
        lab_upper = np.array(p['lab_upper'], dtype=np.uint8)

        hsv_mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        lab_mask = cv2.inRange(lab, lab_lower, lab_upper)

        # ── Step 5: Bitwise AND — combine all masks ───────────────────────
        combined_mask = cv2.bitwise_and(hsv_mask, lab_mask)

        # ── Step 6: Morphological cleanup ─────────────────────────────────
        k_size = p['morph_kernel_size']
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (k_size, k_size)
        )
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN,  kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)

        # ── Find main pear contour ─────────────────────────────────────────
        cnts_info = cv2.findContours(
            combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = cnts_info[0] if len(cnts_info) == 2 else cnts_info[1]

        if not contours:
            return self._no_pear_result()

        main_pear = max(contours, key=cv2.contourArea)
        pear_area = cv2.contourArea(main_pear)

        if pear_area < p['min_pear_area_px']:
            return self._no_pear_result()

        # Pear centroid
        M = cv2.moments(main_pear)
        pear_cx = M['m10'] / M['m00'] if M['m00'] != 0 else 0.0
        pear_cy = M['m01'] / M['m00'] if M['m00'] != 0 else 0.0

        # ── Step 7: Otsu's thresholding on pear-masked grayscale ──────────
        pear_mask = np.zeros(combined_mask.shape, dtype=np.uint8)
        cv2.drawContours(pear_mask, [main_pear], -1, 255, -1)

        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
        masked_gray = cv2.bitwise_and(gray, gray, mask=pear_mask)

        # Run Otsu only on pear pixels (flatten to 1D)
        pear_pixels = masked_gray[pear_mask == 255]
        if len(pear_pixels) == 0:
            return self._no_pear_result()

        otsu_val, _ = cv2.threshold(
            pear_pixels, 0, 255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        # Infection = pixels darker than (otsu_scale × otsu_val)
        scaled_thresh = otsu_val * p['otsu_scale']
        _, inf_mask = cv2.threshold(
            masked_gray, scaled_thresh, 255, cv2.THRESH_BINARY_INV
        )
        inf_mask = cv2.bitwise_and(inf_mask, pear_mask)

        # Second morphology pass to clean up infection blobs
        inf_mask = cv2.morphologyEx(inf_mask, cv2.MORPH_CLOSE, kernel)

        # ── Step 8: Contour analysis on infection regions ─────────────────
        inf_cnts_info = cv2.findContours(
            inf_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        inf_contours = inf_cnts_info[0] if len(inf_cnts_info) == 2 else inf_cnts_info[1]

        inf_area   = sum(cv2.contourArea(c) for c in inf_contours)
        inf_ratio  = inf_area / pear_area if pear_area > 0 else 0.0
        is_infected = inf_ratio > p['infection_ratio_threshold']

        # Infection centroid
        inf_cx, inf_cy = 0.0, 0.0
        if inf_contours:
            largest_inf = max(inf_contours, key=cv2.contourArea)
            M_inf = cv2.moments(largest_inf)
            if M_inf['m00'] != 0:
                inf_cx = M_inf['m10'] / M_inf['m00']
                inf_cy = M_inf['m01'] / M_inf['m00']

        # ── Dominant infection colour (mean BGR of infection pixels) ───────
        inf_r, inf_g, inf_b = 0, 0, 0
        if is_infected and np.any(inf_mask):
            mean_bgr = cv2.mean(color_img, mask=inf_mask)
            inf_b = int(mean_bgr[0])
            inf_g = int(mean_bgr[1])
            inf_r = int(mean_bgr[2])

        # ── Step 9: Return structured result ──────────────────────────────
        return {
            'pear_detected':   True,
            'is_infected':     is_infected,
            'infection_ratio': inf_ratio,
            'infection_area_px': inf_area,
            'pear_area_px':    pear_area,
            'infection_x':     inf_cx,
            'infection_y':     inf_cy,
            'infection_r':     inf_r,
            'infection_g':     inf_g,
            'infection_b':     inf_b,
            'pear_cx':         pear_cx,
            'pear_cy':         pear_cy,
        }

    @staticmethod
    def _no_pear_result() -> dict:
        """Return a zeroed result dict when no valid pear is detected."""
        return {
            'pear_detected':    False,
            'is_infected':      False,
            'infection_ratio':  0.0,
            'infection_area_px': 0.0,
            'pear_area_px':     0.0,
            'infection_x':      0.0,
            'infection_y':      0.0,
            'infection_r':      0,
            'infection_g':      0,
            'infection_b':      0,
            'pear_cx':          0.0,
            'pear_cy':          0.0,
        }


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
