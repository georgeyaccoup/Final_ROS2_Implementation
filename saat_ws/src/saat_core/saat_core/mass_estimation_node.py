#!/usr/bin/env python3
"""
mass_estimation_node
=====================
Converts pear volume (cm³) to mass (g) using a standard pear density model.

  mass_g = volume_cm3 × pear_density_g_cm3

Subscribes
----------
/{zone_id}/volume    std_msgs/Float32   (cm³, per zone)

Publishes
---------
/{zone_id}/mass      std_msgs/Float32   (grams, per zone)
/area_volume_mass    std_msgs/String    (JSON aggregate for IoT + DB)
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

_ZONES = ["A1", "A2", "A3", "B1", "B2", "B3"]


class MassEstimationNode(Node):
    """Converts zone volumes to masses and publishes the IoT aggregate."""

    def __init__(self):
        super().__init__('mass_estimation_node')

        self.declare_parameter('pear_density_g_cm3', 0.96)
        self._density: float = self.get_parameter('pear_density_g_cm3').value

        qos = rclpy.qos.QoSProfile(depth=1)
        self._volumes: dict[str, float] = {z: 0.0 for z in _ZONES}
        self._mass_pubs: dict[str, rclpy.publisher.Publisher] = {}

        # IoT aggregate publisher
        self._iot_pub = self.create_publisher(
            String, '/area_volume_mass', rclpy.qos.QoSProfile(depth=10)
        )

        for zone in _ZONES:
            self._mass_pubs[zone] = self.create_publisher(
                Float32, f'/{zone}/mass', qos
            )
            self.create_subscription(
                Float32,
                f'/{zone}/volume',
                self._make_cb(zone),
                qos
            )

        self.get_logger().info(
            f'mass_estimation_node ready. density={self._density} g/cm³'
        )

    def _make_cb(self, zone: str):
        def _cb(msg: Float32):
            volume = msg.data
            mass   = volume * self._density

            self._volumes[zone] = volume

            out = Float32()
            out.data = float(mass)
            self._mass_pubs[zone].publish(out)

            # Publish aggregate JSON for IoT fields 10–11
            payload = {
                'zone':       zone,
                'volume_cm3': volume,
                'mass_g':     mass,
            }
            s = String()
            s.data = json.dumps(payload)
            self._iot_pub.publish(s)
        return _cb


def main(args=None):
    rclpy.init(args=args)
    node = MassEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
