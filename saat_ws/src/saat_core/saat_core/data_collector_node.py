#!/usr/bin/env python3
"""
data_collector_node  (A1_data_collector … B3_data_collector)
=============================================================
One instance per belt zone.

Responsibilities (Section 11 — 6 tasks)
-----------------------------------------
1. Generate unique Pear ID  →  e.g. "A1_00001"
2. Retrieve infection colour from InfectionResult
3. Retrieve infection area from InfectionResult
4. Retrieve infection location from InfectionResult
5. Compute pear surface area (from area_node)
6. Send all assembled data to data_collection_node

Subscribes
----------
/{zone_id}/detection       saat_interfaces/InfectionResult
/{zone_id}/area            std_msgs/Float32   (2D pear surface area in px²)
/{zone_id}/action          std_msgs/String    ("ACCEPTED"|"REJECTED"|"IDLE")

Publishes
---------
/{zone_id}/pear_data       saat_interfaces/PearData
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from saat_interfaces.msg import InfectionResult, PearData


class DataCollectorNode(Node):
    """Assembles per-pear inspection data and forwards it to the DB node."""

    def __init__(self):
        super().__init__('data_collector_node')

        self.declare_parameter('zone_id', 'A1')
        self._zone = self.get_parameter('zone_id').value

        # ── Pear ID counter ───────────────────────────────────────────────
        self._counter: int = 0

        # ── Latest state from sibling nodes ───────────────────────────────
        self._last_area:   float = 0.0
        self._last_action: str   = 'IDLE'

        # ── QoS ──────────────────────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(depth=1)
        pub_qos = rclpy.qos.QoSProfile(depth=10)

        # ── Publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(
            PearData, f'/{self._zone}/pear_data', pub_qos
        )

        # ── Subscribers ───────────────────────────────────────────────────
        # Area arrives slightly before or after detection; cache it.
        self.create_subscription(
            Float32,
            f'/{self._zone}/area',
            lambda msg: setattr(self, '_last_area', msg.data),
            qos
        )
        self.create_subscription(
            String,
            f'/{self._zone}/action',
            lambda msg: setattr(self, '_last_action', msg.data),
            qos
        )
        # Detection is the trigger — fires last after action is published
        self.create_subscription(
            InfectionResult,
            f'/{self._zone}/detection',
            self._detection_callback,
            qos
        )

        self.get_logger().info(f'[{self._zone}] data_collector_node ready.')

    # ── Callback ──────────────────────────────────────────────────────────
    def _detection_callback(self, msg: InfectionResult) -> None:
        if not msg.pear_detected:
            return   # No pear in this frame — nothing to record

        self._counter += 1
        pear_id = f'{self._zone}_{self._counter:05d}'

        # Classify size by area
        category = 'BIG' if self._last_area >= 15000 else 'SMALL'

        out = PearData()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = self._zone
        out.pear_id         = pear_id
        out.zone_id         = self._zone
        out.pear_status     = self._last_action   # ACCEPTED | REJECTED
        out.pear_category   = category

        out.infection_area_px = msg.infection_area_px
        out.infection_x       = msg.infection_x
        out.infection_y       = msg.infection_y
        out.infection_r       = msg.infection_r
        out.infection_g       = msg.infection_g
        out.infection_b       = msg.infection_b
        out.infection_ratio   = msg.infection_ratio

        out.pear_surface_area_px = self._last_area
        # Volume and mass are filled in by data_collection_node after
        # it merges with the area/volume/mass pipeline.
        out.pear_volume_cm3  = 0.0
        out.pear_mass_g      = 0.0

        self._pub.publish(out)
        self.get_logger().info(
            f'[{self._zone}] Pear {pear_id} → {self._last_action} '
            f'| inf_ratio={msg.infection_ratio:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
