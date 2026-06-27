#!/usr/bin/env python3
"""
action_node  (A1_action … B3_action)
======================================
One instance per belt zone.

Responsibilities
----------------
* Subscribes to the zone's /{zone_id}/detection topic.
* If a pear is detected:
    - REJECTED → rotate servo to reject_angle  (default 90°)
    - ACCEPTED → hold servo at accept_angle    (default 0°)
* After `return_delay_s` seconds, servo returns to accept_angle.
* Controls the PCA9685 channel mapped to this zone via adafruit_servokit.

PCA9685 Channel Map (from servo_control.py reference):
    A1 → channel 0
    A2 → channel 1
    A3 → channel 2
    B1 → channel 3
    B2 → channel 4
    B3 → channel 5

Subscribes
----------
/{zone_id}/detection    saat_interfaces/InfectionResult

Publishes
---------
/{zone_id}/action       std_msgs/String    ("ACCEPTED" | "REJECTED" | "IDLE")
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from saat_interfaces.msg import InfectionResult

try:
    from adafruit_servokit import ServoKit
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False


# Shared PCA9685 driver instance — all 6 action nodes share the same I2C bus.
# Thread-safe because each writes to a different channel.
_SERVO_KIT: 'ServoKit | None' = None
_KIT_LOCK = threading.Lock()


def _get_servokit() -> 'ServoKit | None':
    """Lazily initialise the shared ServoKit singleton."""
    global _SERVO_KIT
    if _SERVO_KIT is None and _HW_AVAILABLE:
        with _KIT_LOCK:
            if _SERVO_KIT is None:          # double-checked locking
                try:
                    _SERVO_KIT = ServoKit(channels=16)
                except Exception as exc:
                    # Not on Jetson Nano hardware — continue in simulation mode.
                    print(f'[action_node] ServoKit init failed: {exc} — running in SIM mode.')
    return _SERVO_KIT


class ActionNode(Node):
    """Commands the PCA9685 servo for one belt zone based on vision result."""

    def __init__(self):
        super().__init__('action_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('zone_id',       'A1')
        self.declare_parameter('pca_channel',   0)
        self.declare_parameter('accept_angle',  0.0)
        self.declare_parameter('reject_angle',  90.0)
        self.declare_parameter('return_delay_s', 0.4)

        self._zone         = self.get_parameter('zone_id').value
        self._channel      = self.get_parameter('pca_channel').value
        self._accept_angle = self.get_parameter('accept_angle').value
        self._reject_angle = self.get_parameter('reject_angle').value
        self._return_delay = self.get_parameter('return_delay_s').value

        # ── Servo driver ──────────────────────────────────────────────────
        self._kit = _get_servokit()
        if self._kit is None:
            self.get_logger().warn(
                f'[{self._zone}] ServoKit unavailable — actions will be simulated.'
            )
        else:
            # Initialise servo to accept position at startup
            self._set_angle(self._accept_angle)
            self.get_logger().info(
                f'[{self._zone}] Servo on channel {self._channel} ready at {self._accept_angle}°'
            )

        # ── Publisher (for motor status / IoT) ────────────────────────────
        qos = rclpy.qos.QoSProfile(depth=10)
        self._pub = self.create_publisher(String, f'/{self._zone}/action', qos)

        # ── Subscriber ────────────────────────────────────────────────────
        self.create_subscription(
            InfectionResult,
            f'/{self._zone}/detection',
            self._detection_callback,
            rclpy.qos.QoSProfile(depth=1)
        )

        self.get_logger().info(f'[{self._zone}] action_node ready.')

    # ── Servo helpers ─────────────────────────────────────────────────────
    def _set_angle(self, angle: float) -> None:
        """Write angle to the PCA9685 channel. Clamps to 0–180°."""
        angle = float(max(0.0, min(180.0, angle)))
        if self._kit is not None:
            try:
                self._kit.servo[self._channel].angle = angle
            except Exception as exc:
                self.get_logger().error(
                    f'[{self._zone}] Servo write error on ch{self._channel}: {exc}'
                )
        else:
            self.get_logger().debug(
                f'[{self._zone}] SIM: servo ch{self._channel} → {angle}°'
            )

    def _return_to_accept(self) -> None:
        """Called in a background thread after return_delay_s."""
        time.sleep(self._return_delay)
        self._set_angle(self._accept_angle)
        self.get_logger().debug(f'[{self._zone}] Servo returned to accept position.')

    # ── Detection callback ────────────────────────────────────────────────
    def _detection_callback(self, msg: InfectionResult) -> None:
        """
        React to vision result within the 1-second action cycle.

        Decision logic:
          - No pear detected → publish IDLE, do not move servo.
          - Pear detected + infected → REJECT, move to reject_angle,
            schedule return after return_delay_s.
          - Pear detected + clean → ACCEPT, hold at accept_angle.
        """
        action_str = String()

        if not msg.pear_detected:
            action_str.data = 'IDLE'
            self._pub.publish(action_str)
            return

        if msg.is_infected:
            action_str.data = 'REJECTED'
            self._set_angle(self._reject_angle)
            # Return servo asynchronously so we don't block the ROS2 spin thread
            t = threading.Thread(target=self._return_to_accept, daemon=True)
            t.start()
            self.get_logger().info(
                f'[{self._zone}] REJECT — ratio {msg.infection_ratio:.3f} '
                f'> threshold | servo → {self._reject_angle}°'
            )
        else:
            action_str.data = 'ACCEPTED'
            self._set_angle(self._accept_angle)
            self.get_logger().debug(
                f'[{self._zone}] ACCEPT — ratio {msg.infection_ratio:.3f}'
            )

        self._pub.publish(action_str)


def main(args=None):
    rclpy.init(args=args)
    node = ActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: return all servos to accept position on shutdown
        kit = _get_servokit()
        if kit is not None:
            for ch in range(6):
                try:
                    kit.servo[ch].angle = 0.0
                except Exception:
                    pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
