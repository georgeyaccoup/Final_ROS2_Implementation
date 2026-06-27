#!/usr/bin/env python3
"""
vision_init_node  (classical_vision_initialization)
====================================================
Runs ONCE at system startup.

Responsibilities
----------------
* Loads all vision algorithm parameters from the ROS2 parameter server.
* Publishes them as a latched JSON string on /vision_params so that all 6
  vision nodes can pick them up even if they start later than this node.
* Logs a confirmation once params are broadcast.

CRITICAL (Section 17): All 6 vision nodes must receive this message before
processing their first frame.  The launch file enforces start ordering via
a condition on this node's /vision_params topic.

Publishes
---------
/vision_params    std_msgs/String    (JSON, latched QoS)
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


class VisionInitNode(Node):
    """Broadcasts vision algorithm parameters to the parameter bus."""

    def __init__(self):
        super().__init__('vision_init_node')

        # ── Declare all vision parameters ─────────────────────────────────
        self.declare_parameter('hsv_lower',                 [10, 40, 40])
        self.declare_parameter('hsv_upper',                 [95, 255, 255])
        self.declare_parameter('lab_lower',                 [50, 120, 130])
        self.declare_parameter('lab_upper',                 [255, 150, 200])
        self.declare_parameter('max_depth_mm',              380)
        self.declare_parameter('infection_ratio_threshold', 0.03)
        self.declare_parameter('otsu_scale',                0.75)
        self.declare_parameter('morph_kernel_size',         3)
        self.declare_parameter('min_pear_area_px',          500)
        self.declare_parameter('clahe_clip_limit',          2.0)
        self.declare_parameter('clahe_tile_size',           [8, 8])
        self.declare_parameter('bilateral_d',               9)
        self.declare_parameter('bilateral_sigma_color',     75.0)
        self.declare_parameter('bilateral_sigma_space',     75.0)

        # ── Build the params dict ─────────────────────────────────────────
        params = {
            'hsv_lower':                 self.get_parameter('hsv_lower').value,
            'hsv_upper':                 self.get_parameter('hsv_upper').value,
            'lab_lower':                 self.get_parameter('lab_lower').value,
            'lab_upper':                 self.get_parameter('lab_upper').value,
            'max_depth_mm':              self.get_parameter('max_depth_mm').value,
            'infection_ratio_threshold': self.get_parameter('infection_ratio_threshold').value,
            'otsu_scale':                self.get_parameter('otsu_scale').value,
            'morph_kernel_size':         self.get_parameter('morph_kernel_size').value,
            'min_pear_area_px':          self.get_parameter('min_pear_area_px').value,
            'clahe_clip_limit':          self.get_parameter('clahe_clip_limit').value,
            'clahe_tile_size':           list(self.get_parameter('clahe_tile_size').value),
            'bilateral_d':               self.get_parameter('bilateral_d').value,
            'bilateral_sigma_color':     self.get_parameter('bilateral_sigma_color').value,
            'bilateral_sigma_space':     self.get_parameter('bilateral_sigma_space').value,
        }

        # ── Latched publisher (TRANSIENT_LOCAL) ───────────────────────────
        # Any late-joining subscriber will still receive this message.
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        pub = self.create_publisher(String, '/vision_params', latch_qos)

        msg = String()
        msg.data = json.dumps(params)
        pub.publish(msg)

        self.get_logger().info('Vision parameters broadcast on /vision_params:')
        for k, v in params.items():
            self.get_logger().info(f'  {k}: {v}')

        self.get_logger().info('classical_vision_initialization complete.')


def main(args=None):
    rclpy.init(args=args)
    node = VisionInitNode()
    # Spin briefly to allow the latched message to be delivered, then exit.
    # The TRANSIENT_LOCAL QoS ensures late subscribers still receive it.
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
