#!/usr/bin/env python3
"""
main_speed_node  →  centroid_time_speed
========================================
Implements the 7-step Reference Speed Algorithm (Section 10).

Algorithm Steps
---------------
1. Find pears in the vision-zone centre strip (cols 257–1024).
2. Check pear count → overflow if > max_pears.
3. Find centroids for each pear.
4. Track centroids across consecutive frames (t₁ → t₂).
5. Compute per-pear pixel speed → convert to m/s.
6. Average speed over all *moving* pears.
7. Publish reference speed + belt state.

Uses the colour sub-frame from /speed_frame/vision (not the full frame)
for centroid tracking.  Also aggregates InfectionResult centroid outputs
as a secondary, higher-quality source for pear detection.

Subscribes
----------
/speed_frame/vision          sensor_msgs/Image      (centre column strip)
/A1/detection … /B3/detection  saat_interfaces/InfectionResult  (centroid pool)

Publishes
---------
/main_speed                  saat_interfaces/SpeedCommand
/centroid_time_speed         std_msgs/Float32       (reference speed in m/s)
"""

import math
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

from saat_interfaces.msg import InfectionResult, SpeedCommand

_ZONES = ["A1", "A2", "A3", "B1", "B2", "B3"]

# Minimum voltage guardrail (Section 17 / Section 5)
_MIN_V = 0.1
_MAX_V = 3.3


def _clamp_voltage(v: float) -> float:
    return max(_MIN_V, min(_MAX_V, v))


class MainSpeedNode(Node):
    """Computes the belt reference speed from pear centroid tracking."""

    def __init__(self):
        super().__init__('main_speed_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('px_to_m',           0.0005)
        self.declare_parameter('camera_fps',         30.0)
        self.declare_parameter('max_pears',          6)
        self.declare_parameter('min_displacement_px', 5.0)
        self.declare_parameter('use_3d_speed',       True)

        self._px_to_m    = self.get_parameter('px_to_m').value
        self._fps        = self.get_parameter('camera_fps').value
        self._max_pears  = self.get_parameter('max_pears').value
        self._min_disp   = self.get_parameter('min_displacement_px').value

        # ── Centroid tracking state ────────────────────────────────────────
        # List of (cx, cy, timestamp) for tracked pears from previous frame
        self._prev_centroids: list[tuple[float, float, float]] = []

        # Pool of centroids received from vision nodes (per zone, latest only)
        self._zone_centroids: dict[str, tuple[float, float] | None] = {
            z: None for z in _ZONES
        }

        # ── Bridge & QoS ──────────────────────────────────────────────────
        self._bridge = CvBridge()
        qos = rclpy.qos.QoSProfile(depth=1)

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_cmd = self.create_publisher(SpeedCommand, '/main_speed', qos)
        self._pub_ref = self.create_publisher(Float32, '/centroid_time_speed', qos)

        # ── Subscribers ───────────────────────────────────────────────────
        # Primary: vision-zone colour strip for CV-based tracking
        self.create_subscription(
            Image,
            '/speed_frame/vision',
            self._frame_callback,
            qos
        )
        # Secondary: centroid data from each vision node
        for zone in _ZONES:
            self.create_subscription(
                InfectionResult,
                f'/{zone}/detection',
                self._make_detection_cb(zone),
                qos
            )

        self.get_logger().info('main_speed_node ready.')

    # ── Detection callbacks (secondary centroid source) ───────────────────
    def _make_detection_cb(self, zone: str):
        def _cb(msg: InfectionResult):
            if msg.pear_detected:
                self._zone_centroids[zone] = (msg.pear_centroid_x, msg.pear_centroid_y)
            else:
                self._zone_centroids[zone] = None
        return _cb

    # ── Primary frame callback ────────────────────────────────────────────
    def _frame_callback(self, msg: Image) -> None:
        """
        Detect pears in the vision strip, track centroids frame-to-frame,
        compute reference speed, and publish SpeedCommand + Float32.
        """
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        now = time.monotonic()

        # ── Step 1 & 3: Detect pears and find centroids ───────────────────
        current_centroids = self._detect_centroids(img)

        # Merge with vision-node centroid pool (adds pears from all 6 zones)
        for zone, centroid in self._zone_centroids.items():
            if centroid is not None:
                current_centroids.append(centroid)

        # Deduplicate centroids that are very close together
        current_centroids = self._deduplicate(current_centroids, threshold_px=40.0)

        pear_count = len(current_centroids)

        # ── Step 2: Overflow check ────────────────────────────────────────
        belt_state: str
        ref_speed: float
        conv1_v: float
        conv2_v: float

        if pear_count == 0:
            # EMPTY: pull more pears in
            belt_state = 'EMPTY'
            ref_speed  = 0.5              # max speed (maps to 3.3 V)
            conv1_v    = _clamp_voltage(_MAX_V)
            conv2_v    = _clamp_voltage(_MIN_V)

        elif pear_count > self._max_pears:
            # CROWDED: clear the queue
            belt_state = 'CROWDED'
            ref_speed  = 0.5
            conv1_v    = _clamp_voltage(_MIN_V)
            conv2_v    = _clamp_voltage(_MAX_V)

        else:
            # NORMAL: compute actual reference speed from tracking
            belt_state = 'NORMAL'
            ref_speed  = self._compute_reference_speed(
                current_centroids, now
            )
            # Convert m/s → voltage:  0 m/s → 0.1 V,  0.5 m/s → 3.3 V
            lv = (ref_speed / 0.5) * _MAX_V
            conv2_v = _clamp_voltage(lv)
            conv1_v = _clamp_voltage(_MAX_V - conv2_v)

        # ── Step 4: Store this frame's centroids for next iteration ───────
        self._prev_centroids = [
            (cx, cy, now) for (cx, cy) in current_centroids
        ]

        # ── Step 7: Publish ───────────────────────────────────────────────
        cmd = SpeedCommand()
        cmd.header.stamp        = msg.header.stamp
        cmd.reference_speed_ms  = float(ref_speed)
        cmd.conv1_voltage       = float(conv1_v)
        cmd.conv2_voltage       = float(conv2_v)
        cmd.servo_voltage       = float(conv2_v)   # servo tracks conv2 by default
        cmd.belt_state          = belt_state
        cmd.pear_count          = pear_count
        self._pub_cmd.publish(cmd)

        ref_msg = Float32()
        ref_msg.data = float(ref_speed)
        self._pub_ref.publish(ref_msg)

    # ── Centroid detection (CV-based, on vision strip) ────────────────────
    def _detect_centroids(self, img: np.ndarray) -> list[tuple[float, float]]:
        """
        Simple HSV-based pear detection on the vision strip.
        Returns list of (cx, cy) centroids.
        """
        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([10, 40, 40],   dtype=np.uint8),
            np.array([95, 255, 255], dtype=np.uint8)
        )
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        cnts_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours  = cnts_info[0] if len(cnts_info) == 2 else cnts_info[1]

        centroids = []
        for c in contours:
            if cv2.contourArea(c) < 500:
                continue
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = M['m10'] / M['m00']
                cy = M['m01'] / M['m00']
                centroids.append((cx, cy))

        return centroids

    # ── Reference speed computation ───────────────────────────────────────
    def _compute_reference_speed(
        self,
        current: list[tuple[float, float]],
        now: float
    ) -> float:
        """
        Steps 4–6: Track centroids from previous frame, compute per-pear
        velocity, return the average speed of all *moving* pears (m/s).
        """
        if not self._prev_centroids or not current:
            return 0.0

        speeds: list[float] = []

        for cx, cy in current:
            # Find the nearest centroid in the previous frame
            best_dist = float('inf')
            best_prev = None
            for px, py, pt in self._prev_centroids:
                d = math.hypot(cx - px, cy - py)
                if d < best_dist:
                    best_dist = d
                    best_prev = (px, py, pt)

            if best_prev is None:
                continue

            px, py, pt = best_prev
            dt = now - pt
            if dt <= 0:
                continue

            displacement_px = math.hypot(cx - px, cy - py)

            # Only count pears that are actually moving
            if displacement_px < self._min_disp:
                continue

            speed_px_s = displacement_px / dt
            speed_ms   = speed_px_s * self._px_to_m
            speeds.append(speed_ms)

        if not speeds:
            return 0.0

        # Step 6: Average over all moving pears (Section 10)
        return float(sum(speeds) / len(speeds))

    @staticmethod
    def _deduplicate(
        centroids: list[tuple[float, float]],
        threshold_px: float
    ) -> list[tuple[float, float]]:
        """Remove duplicate centroids that are closer than threshold_px."""
        unique: list[tuple[float, float]] = []
        for cx, cy in centroids:
            if all(
                math.hypot(cx - ux, cy - uy) > threshold_px
                for ux, uy in unique
            ):
                unique.append((cx, cy))
        return unique


def main(args=None):
    rclpy.init(args=args)
    node = MainSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
