#!/usr/bin/env python3
"""
speed_publisher_node
====================
The hardware interface between the ROS2 speed pipeline and the PLC.

Responsibilities
----------------
* Runs two independent software-PWM threads (one per conveyor GPIO pin).
* Implements a PID loop for each conveyor channel.
* Enforces the hardware constraint: Conv1_V + Conv2_V = 3.3 V always.
* Enforces minimum voltage: no output ever drops below 0.1 V.
* Publishes the commanded voltages back to /speed_to_plc for monitoring.

Software PWM (adapted from voltage_publisher.py)
-------------------------------------------------
  GPIO pin 11 (BOARD) → Conv1 → Low-Pass Filter → PLC channel A
  GPIO pin 13 (BOARD) → Conv2 → Low-Pass Filter → PLC channel B
  Frequency: 500 Hz
  Duty cycle: requested_voltage / 3.3

Subscribes
----------
/main_speed          saat_interfaces/SpeedCommand   (target voltages)
/conv1_speed_feedback  std_msgs/Float32              (actual conv1 speed m/s)
/conv2_speed_feedback  std_msgs/Float32              (actual conv2 speed m/s)

Publishes
---------
/speed_to_plc        saat_interfaces/SpeedCommand   (commanded + actual)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from saat_interfaces.msg import SpeedCommand

# ── Jetson.GPIO import with graceful fallback for non-Nano environments ────
try:
    import Jetson.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


_MIN_V = 0.1
_MAX_V = 3.3


def _clamp(v: float) -> float:
    return max(_MIN_V, min(_MAX_V, v))


# ── PID Controller (same as servo_speed_node, kept self-contained) ─────────
class _PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_t     = time.monotonic()

    def compute(self, setpoint: float, feedback: float) -> float:
        now = time.monotonic()
        dt  = max(now - self._prev_t, 1e-6)
        err = setpoint - feedback

        self._integral   = max(-10.0, min(10.0, self._integral + err * dt))
        derivative       = (err - self._prev_error) / dt
        out              = self.kp * err + self.ki * self._integral + self.kd * derivative
        self._prev_error = err
        self._prev_t     = now
        return out

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_t     = time.monotonic()


# ── PWM Channel ────────────────────────────────────────────────────────────
class _PWMChannel:
    """
    Software PWM worker thread for one GPIO pin.
    Exactly mirrors the logic of voltage_publisher.py but as a class.
    """

    def __init__(self, pin: int, freq: int = 500, max_v: float = 3.3):
        self._pin      = pin
        self._period   = 1.0 / freq
        self._max_v    = max_v
        self._duty     = 0.0          # 0.0–1.0
        self._running  = False
        self._lock     = threading.Lock()
        self._thread: threading.Thread | None = None

    # ── Public API ────────────────────────────────────────────────────────
    def set_voltage(self, voltage: float) -> None:
        """Update requested output voltage (thread-safe)."""
        v = max(0.0, min(self._max_v, voltage))
        with self._lock:
            self._duty = v / self._max_v

    def start(self) -> None:
        if not _GPIO_AVAILABLE:
            return
        GPIO.setup(self._pin, GPIO.OUT)
        self._running = True
        self._thread  = threading.Thread(
            target=self._run, daemon=True, name=f'pwm_pin{self._pin}'
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if _GPIO_AVAILABLE:
            GPIO.output(self._pin, GPIO.LOW)
        if self._thread:
            self._thread.join(timeout=1.0)

    # ── Worker loop ───────────────────────────────────────────────────────
    def _run(self) -> None:
        """
        Identical logic to voltage_publisher.py::generate_pwm().
        Runs in its own daemon thread at 500 Hz.
        """
        while self._running:
            with self._lock:
                duty = self._duty

            if duty <= 0.0:
                GPIO.output(self._pin, GPIO.LOW)
                time.sleep(0.01)
            elif duty >= 1.0:
                GPIO.output(self._pin, GPIO.HIGH)
                time.sleep(0.01)
            else:
                time_on  = self._period * duty
                time_off = self._period * (1.0 - duty)
                GPIO.output(self._pin, GPIO.HIGH)
                time.sleep(time_on)
                GPIO.output(self._pin, GPIO.LOW)
                time.sleep(time_off)


# ── Main Node ──────────────────────────────────────────────────────────────
class SpeedPublisherNode(Node):
    """
    Receives target voltages, runs PID correction, and drives dual-channel
    software PWM on GPIO pins 11 and 13.
    """

    def __init__(self):
        super().__init__('speed_publisher_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('gpio_pin_conv1', 11)
        self.declare_parameter('gpio_pin_conv2', 13)
        self.declare_parameter('pwm_frequency',  500)
        self.declare_parameter('max_voltage',    3.3)
        self.declare_parameter('min_voltage',    0.1)

        self.declare_parameter('kp_conv1', 1.0)
        self.declare_parameter('ki_conv1', 0.1)
        self.declare_parameter('kd_conv1', 0.05)
        self.declare_parameter('kp_conv2', 1.0)
        self.declare_parameter('ki_conv2', 0.1)
        self.declare_parameter('kd_conv2', 0.05)

        pin1 = self.get_parameter('gpio_pin_conv1').value
        pin2 = self.get_parameter('gpio_pin_conv2').value
        freq = self.get_parameter('pwm_frequency').value

        # ── GPIO setup ────────────────────────────────────────────────────
        if _GPIO_AVAILABLE:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            self.get_logger().info(f'GPIO available — Pin {pin1}=Conv1, Pin {pin2}=Conv2')
        else:
            self.get_logger().warn(
                'Jetson.GPIO not available — running in SIMULATION mode (no PWM output).'
            )

        # ── PWM channels ─────────────────────────────────────────────────
        self._pwm1 = _PWMChannel(pin1, freq)
        self._pwm2 = _PWMChannel(pin2, freq)
        self._pwm1.start()
        self._pwm2.start()

        # Initialise both conveyors to minimum voltage
        self._pwm1.set_voltage(_MIN_V)
        self._pwm2.set_voltage(_MAX_V - _MIN_V)

        # ── PID loops ─────────────────────────────────────────────────────
        self._pid1 = _PID(
            self.get_parameter('kp_conv1').value,
            self.get_parameter('ki_conv1').value,
            self.get_parameter('kd_conv1').value,
        )
        self._pid2 = _PID(
            self.get_parameter('kp_conv2').value,
            self.get_parameter('ki_conv2').value,
            self.get_parameter('kd_conv2').value,
        )

        # ── State ─────────────────────────────────────────────────────────
        self._target_v1:   float = _MIN_V
        self._target_v2:   float = _MAX_V - _MIN_V
        self._actual_v1:   float = 0.0        # populated from conv speed feedback
        self._actual_v2:   float = 0.0
        self._last_cmd:    SpeedCommand | None = None

        # ── QoS & pub/sub ─────────────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(depth=1)
        self._pub = self.create_publisher(SpeedCommand, '/speed_to_plc', qos)

        self.create_subscription(SpeedCommand, '/main_speed',
                                 self._speed_cmd_cb, qos)
        self.create_subscription(Float32, '/conv1_speed_feedback',
                                 self._feedback1_cb, qos)
        self.create_subscription(Float32, '/conv2_speed_feedback',
                                 self._feedback2_cb, qos)

        # ── Control loop timer (10 Hz — fast enough for PLC response) ─────
        self._timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('speed_publisher_node ready.')

    # ── Callbacks ─────────────────────────────────────────────────────────
    def _speed_cmd_cb(self, msg: SpeedCommand) -> None:
        """Cache the latest speed command from main_speed."""
        self._last_cmd   = msg
        self._target_v1  = _clamp(msg.conv1_voltage)
        self._target_v2  = _clamp(msg.conv2_voltage)
        # Enforce Conv1 + Conv2 = MAX_V
        self._target_v2  = _clamp(_MAX_V - self._target_v1)

    def _feedback1_cb(self, msg: Float32) -> None:
        """Cache actual Conv1 speed (m/s) as PID feedback.
        Convert m/s → equivalent voltage for the PID error term."""
        self._actual_v1 = (msg.data / 0.5) * _MAX_V

    def _feedback2_cb(self, msg: Float32) -> None:
        self._actual_v2 = (msg.data / 0.5) * _MAX_V

    # ── Control loop ──────────────────────────────────────────────────────
    def _control_loop(self) -> None:
        """
        10 Hz PID update.
        Applies PID correction to the target voltages, enforces the
        Conv1 + Conv2 = 3.3V constraint, then writes to PWM channels.
        """
        # PID correction
        corr1 = self._pid1.compute(self._target_v1, self._actual_v1)
        corr2 = self._pid2.compute(self._target_v2, self._actual_v2)

        v1 = _clamp(self._target_v1 + corr1)
        v2 = _clamp(_MAX_V - v1)                 # Hard constraint: V1+V2=3.3V

        # Write to hardware
        self._pwm1.set_voltage(v1)
        self._pwm2.set_voltage(v2)

        if not _GPIO_AVAILABLE:
            self.get_logger().debug(f'SIM PWM → Conv1={v1:.2f}V Conv2={v2:.2f}V')

        # Publish commanded state for monitoring + IoT
        cmd = SpeedCommand()
        cmd.header.stamp   = self.get_clock().now().to_msg()
        if self._last_cmd:
            cmd.reference_speed_ms = self._last_cmd.reference_speed_ms
            cmd.belt_state         = self._last_cmd.belt_state
            cmd.pear_count         = self._last_cmd.pear_count
            cmd.servo_voltage      = self._last_cmd.servo_voltage
        cmd.conv1_voltage = v1
        cmd.conv2_voltage = v2
        self._pub.publish(cmd)

    # ── Shutdown ──────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info('speed_publisher_node shutting down — zeroing PWM…')
        self._pwm1.stop()
        self._pwm2.stop()
        if _GPIO_AVAILABLE:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeedPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
