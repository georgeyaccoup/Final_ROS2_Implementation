#!/usr/bin/env python3
"""
servo_speed_node
================
Runs a PID control loop that maps the reference belt speed to a servo
speed command.  The servo speed determines how fast the ejection arm
sweeps when rejecting an infected pear — faster belt = faster sweep needed.

PID setpoint  : reference_speed_ms  (from /centroid_time_speed)
PID feedback  : current_servo_speed (estimated from action timing)
PID output    : servo_voltage        (0.1 – 3.3 V → PCA9685 via /servo_cmd)

Subscribes
----------
/centroid_time_speed     std_msgs/Float32   (reference speed, m/s)
/main_speed              saat_interfaces/SpeedCommand

Publishes
---------
/servo_cmd               std_msgs/Float32   (voltage 0.1–3.3 V)
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from saat_interfaces.msg import SpeedCommand

_MIN_V = 0.1
_MAX_V = 3.3


class PIDController:
    """Minimal discrete-time PID with anti-windup."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float, output_max: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = output_min
        self.out_max = output_max

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.monotonic()

    def compute(self, setpoint: float, feedback: float) -> float:
        now = time.monotonic()
        dt  = now - self._prev_time
        if dt <= 0:
            return self.out_min

        error         = setpoint - feedback
        self._integral += error * dt
        derivative    = (error - self._prev_error) / dt

        # Anti-windup: clamp integral contribution
        self._integral = max(
            self.out_min / self.ki if self.ki != 0 else -1e6,
            min(self.out_max / self.ki if self.ki != 0 else 1e6,
                self._integral)
        )

        output = (self.kp * error
                  + self.ki * self._integral
                  + self.kd * derivative)
        output = max(self.out_min, min(self.out_max, output))

        self._prev_error = error
        self._prev_time  = now
        return output

    def reset(self) -> None:
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.monotonic()


class ServoSpeedNode(Node):
    """PID controller that converts reference speed to servo voltage."""

    def __init__(self):
        super().__init__('servo_speed_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('kp_servo', 0.8)
        self.declare_parameter('ki_servo', 0.05)
        self.declare_parameter('kd_servo', 0.02)

        kp = self.get_parameter('kp_servo').value
        ki = self.get_parameter('ki_servo').value
        kd = self.get_parameter('kd_servo').value

        self._pid = PIDController(kp, ki, kd, _MIN_V, _MAX_V)

        # Current estimated servo speed (feedback); starts at 0
        self._current_servo_speed: float = 0.0

        qos = rclpy.qos.QoSProfile(depth=1)

        # ── Publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(Float32, '/servo_cmd', qos)

        # ── Subscribers ───────────────────────────────────────────────────
        # Reference speed (setpoint)
        self.create_subscription(
            Float32,
            '/centroid_time_speed',
            self._ref_speed_cb,
            qos
        )
        # SpeedCommand carries servo_voltage as initial feedforward hint
        self.create_subscription(
            SpeedCommand,
            '/main_speed',
            self._speed_cmd_cb,
            qos
        )

        self.get_logger().info(
            f'servo_speed_node ready | Kp={kp} Ki={ki} Kd={kd}'
        )

    def _ref_speed_cb(self, msg: Float32) -> None:
        """Compute PID output from reference speed setpoint."""
        setpoint = msg.data
        output   = self._pid.compute(setpoint, self._current_servo_speed)

        out = Float32()
        out.data = float(output)
        self._pub.publish(out)

        self.get_logger().debug(
            f'ServoSpeed | setpoint={setpoint:.3f} '
            f'feedback={self._current_servo_speed:.3f} '
            f'output={output:.3f} V'
        )

    def _speed_cmd_cb(self, msg: SpeedCommand) -> None:
        """Cache the commanded servo voltage as the feedback estimate."""
        self._current_servo_speed = msg.servo_voltage


def main(args=None):
    rclpy.init(args=args)
    node = ServoSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
