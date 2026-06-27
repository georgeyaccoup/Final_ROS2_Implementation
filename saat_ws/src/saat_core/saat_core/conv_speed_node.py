#!/usr/bin/env python3
"""
conv_speed_node  (conv1_speed_node / conv2_speed_node)
=======================================================
Measures actual conveyor belt speed from the horizontal motion of the
belt surface visible in the speed sub-frame (optical flow line-counting).

Method (Section 7)
------------------
Counts how many times a reference line on the belt crosses a pixel
threshold per second.  Each crossing = one full belt cycle.
  speed_m_s = crossing_count × belt_length_m / elapsed_time

For Docker/Jetson deployment: uses dense optical flow (Lucas-Kanade)
on the Conv column strip rather than a physical encoder, since we read
the belt visually via the RealSense D455.

Subscribes
----------
/speed_frame/conv1   OR   /speed_frame/conv2    sensor_msgs/Image

Publishes
---------
/conv1_speed_feedback   OR   /conv2_speed_feedback   std_msgs/Float32  (m/s)
"""

import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

# Lucas-Kanade optical flow parameters
_LK_PARAMS = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
)
_FEATURE_PARAMS = dict(
    maxCorners=50,
    qualityLevel=0.3,
    minDistance=7,
    blockSize=7
)


class ConvSpeedNode(Node):
    """Estimates belt speed via optical flow on the conveyor column strip."""

    def __init__(self):
        super().__init__('conv_speed_node')

        self.declare_parameter('belt_length_m', 1.75)
        self.declare_parameter('zone_id',       'CONV1')

        self._belt_length: float = self.get_parameter('belt_length_m').value
        self._zone:        str   = self.get_parameter('zone_id').value

        topic_in  = f'/speed_frame/{self._zone.lower()}'
        topic_out = f'/{self._zone.lower()}_speed_feedback'

        self._bridge = CvBridge()

        # ── Optical flow state ────────────────────────────────────────────
        self._prev_gray:    np.ndarray | None = None
        self._prev_pts:     np.ndarray | None = None
        self._prev_time:    float             = 0.0
        self._speed_buffer: list[float]       = []   # rolling average window

        # ── Publisher ─────────────────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(depth=1)
        self._pub = self.create_publisher(Float32, topic_out, qos)

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(Image, topic_in, self._frame_cb, qos)

        self.get_logger().info(
            f'[{self._zone}] conv_speed_node | belt={self._belt_length}m '
            f'| in={topic_in} | out={topic_out}'
        )

    def _frame_cb(self, msg: Image) -> None:
        img  = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        now  = time.monotonic()

        if self._prev_gray is None:
            # First frame — initialise
            self._prev_gray = gray
            self._prev_pts  = cv2.goodFeaturesToTrack(
                gray, mask=None, **_FEATURE_PARAMS
            )
            self._prev_time = now
            return

        if self._prev_pts is None or len(self._prev_pts) == 0:
            # No features — reinitialise
            self._prev_gray = gray
            self._prev_pts  = cv2.goodFeaturesToTrack(
                gray, mask=None, **_FEATURE_PARAMS
            )
            self._prev_time = now
            return

        # ── Lucas-Kanade optical flow ──────────────────────────────────
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **_LK_PARAMS
        )

        if next_pts is None:
            self._prev_gray = gray
            self._prev_pts  = None
            return

        good_prev = self._prev_pts[status == 1]
        good_next = next_pts[status == 1]

        dt = now - self._prev_time
        if dt <= 0 or len(good_prev) == 0:
            self._prev_gray = gray
            self._prev_pts  = good_next.reshape(-1, 1, 2) if len(good_next) else None
            self._prev_time = now
            return

        # Horizontal displacement only (belt moves horizontally in column strip)
        dx_px = float(np.mean(np.abs(good_next[:, 0] - good_prev[:, 0])))

        # px/s → m/s using the known belt scale
        # Calibration: at 30 fps the belt strip is 256 px wide = belt_length_m
        px_per_m = 256.0 / self._belt_length
        speed_ms = (dx_px / dt) / px_per_m

        # Rolling average over last 5 frames for stability
        self._speed_buffer.append(speed_ms)
        if len(self._speed_buffer) > 5:
            self._speed_buffer.pop(0)
        avg_speed = float(np.mean(self._speed_buffer))

        out = Float32()
        out.data = avg_speed
        self._pub.publish(out)

        # Reinitialise features periodically (every 30 frames)
        self._prev_gray = gray
        self._prev_pts  = cv2.goodFeaturesToTrack(
            gray, mask=None, **_FEATURE_PARAMS
        ) if len(good_next) < 10 else good_next.reshape(-1, 1, 2)
        self._prev_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ConvSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
