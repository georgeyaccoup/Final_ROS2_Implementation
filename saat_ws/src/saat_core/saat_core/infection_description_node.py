#!/usr/bin/env python3
"""
infection_description_node
===========================
Aggregates InfectionResult messages from all 6 zones and publishes a
combined, system-wide infection summary to the IoT topic and local DB.

Subscribes
----------
/A1/detection … /B3/detection   saat_interfaces/InfectionResult

Publishes
---------
/infection_description          std_msgs/String   (JSON, one record per infected pear)
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from saat_interfaces.msg import InfectionResult

_ZONES = ["A1", "A2", "A3", "B1", "B2", "B3"]


class InfectionDescriptionNode(Node):
    """Aggregates zone-level infection results into system-wide IoT records."""

    def __init__(self):
        super().__init__('infection_description_node')

        qos = rclpy.qos.QoSProfile(depth=1)
        self._pub = self.create_publisher(String, '/infection_description', rclpy.qos.QoSProfile(depth=10))

        for zone in _ZONES:
            self.create_subscription(
                InfectionResult,
                f'/{zone}/detection',
                self._make_callback(zone),
                qos
            )

        self.get_logger().info('infection_description_node ready — listening to all 6 zones.')

    def _make_callback(self, zone: str):
        def _cb(msg: InfectionResult):
            if not msg.pear_detected or not msg.is_infected:
                return
            record = {
                'zone_id':         zone,
                'infection_area':  msg.infection_area_px,
                'infection_x':     msg.infection_x,
                'infection_y':     msg.infection_y,
                'infection_rgb':   [msg.infection_r, msg.infection_g, msg.infection_b],
                'infection_ratio': msg.infection_ratio,
                'timestamp':       msg.header.stamp.sec,
            }
            out = String()
            out.data = json.dumps(record)
            self._pub.publish(out)
            self.get_logger().info(f'[{zone}] Infection published: ratio={msg.infection_ratio:.3f}')
        return _cb


def main(args=None):
    rclpy.init(args=args)
    node = InfectionDescriptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
