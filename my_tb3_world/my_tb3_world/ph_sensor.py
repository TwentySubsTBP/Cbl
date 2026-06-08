#!/usr/bin/env python3
"""Simulated water-quality sensor for the WaveRider digital twin (FR1.1, STATE SYNC).

Course rule: no extra *physical* sensor on the robot. This node is a pure
software stand-in for WaveRider's pH/CO2/temperature probe. It tracks the
robot's position from /odom and publishes plausible, position-dependent
readings on /water_quality so the digital twin can sync state and flag
anomalies.

The readings drift smoothly across the arena (so the DT sees a spatial field,
not a constant), and a scriptable "sudden drop" anomaly can be triggered at
demo time to mimic an acidification spike / pollution leak:

  # start the sim + this node, then in another terminal:
  ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: true}"   # leak ON
  ros2 topic pub --once /ph_anomaly std_msgs/Bool "{data: false}"  # leak OFF

Or have it auto-clear after a few seconds via the anomaly_hold parameter.

Run:
  ros2 run my_tb3_world ph_sensor
  ros2 run my_tb3_world ph_sensor --ros-args -p rate_hz:=5.0 -p anomaly_ph:=6.2

Interface contract (see HANDOFF): /water_quality is std_msgs/String carrying
JSON, matching obstacle_info. Consumed by dt_supervisor / anomaly_alert.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry


class PhSensor(Node):
    def __init__(self):
        super().__init__('ph_sensor')

        # --- Tunables (override at runtime with --ros-args -p name:=value) ---
        self.rate_hz = float(self.declare_parameter('rate_hz', 2.0).value)
        self.base_ph = float(self.declare_parameter('base_ph', 8.1).value)      # healthy seawater ~8.1
        self.base_co2 = float(self.declare_parameter('base_co2', 410.0).value)  # ppm, atmospheric-ish
        self.base_temp = float(self.declare_parameter('base_temp', 15.0).value)  # deg C
        # Where pH drops to while a leak/anomaly is active (acidification spike).
        self.anomaly_ph = float(self.declare_parameter('anomaly_ph', 6.0).value)
        # If > 0, an anomaly toggled ON auto-clears after this many seconds.
        # If <= 0, the anomaly stays on until explicitly toggled off.
        self.anomaly_hold = float(self.declare_parameter('anomaly_hold', 0.0).value)

        # /odom is published BEST_EFFORT by the gz bridge -> match QoS.
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pose = None              # (x, y) once odom arrives
        self.anomaly = False          # leak / acidification flag
        self._anomaly_until = None    # wall-clock deadline when auto-clearing

        self.create_subscription(Odometry, 'odom', self._on_odom, sensor_qos)
        # Anomaly trigger is a normal control topic -> default (reliable) QoS.
        self.create_subscription(Bool, 'ph_anomaly', self._on_anomaly, 10)

        self.pub = self.create_publisher(String, 'water_quality', sensor_qos)

        period = 1.0 / self.rate_hz if self.rate_hz > 0 else 0.5
        self.create_timer(period, self._tick)
        self.get_logger().info(
            'ph_sensor up: publishing /water_quality at %.1f Hz '
            '(base pH %.2f, anomaly pH %.2f). Trigger a leak with '
            '/ph_anomaly std_msgs/Bool.' % (self.rate_hz, self.base_ph, self.anomaly_ph))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _on_anomaly(self, msg):
        self.anomaly = bool(msg.data)
        if self.anomaly and self.anomaly_hold > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._anomaly_until = now + self.anomaly_hold
        else:
            self._anomaly_until = None
        self.get_logger().warn(
            'pH anomaly %s' % ('TRIGGERED (leak/acidification)' if self.anomaly else 'cleared'))

    def _field(self, x, y):
        """Smooth, position-dependent water-quality field across the arena.

        Returns (ph, co2, temp). pH dips slightly toward the arena edges and a
        far corner; CO2 rises as pH falls (carbonate chemistry), temp drifts
        mildly. Values are illustrative, not physically calibrated.
        """
        # Gentle spatial variation, bounded to a few hundredths of a pH unit.
        ph = self.base_ph + 0.05 * math.sin(0.8 * x) - 0.04 * math.cos(0.6 * y)
        # Lower pH -> more dissolved CO2 (acidification). ~80 ppm per pH unit.
        co2 = self.base_co2 + (self.base_ph - ph) * 80.0
        temp = self.base_temp + 0.3 * math.sin(0.4 * x + 0.5 * y)
        return ph, co2, temp

    def _tick(self):
        if self.pose is None:
            return  # wait for the first /odom so position is meaningful

        # Auto-clear a held anomaly once its window elapses.
        if self._anomaly_until is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now >= self._anomaly_until:
                self.anomaly = False
                self._anomaly_until = None
                self.get_logger().warn('pH anomaly auto-cleared (hold elapsed)')

        x, y = self.pose
        ph, co2, temp = self._field(x, y)
        if self.anomaly:
            ph = self.anomaly_ph
            co2 = self.base_co2 + (self.base_ph - ph) * 80.0

        reading = {
            'ph': round(ph, 3),
            'co2_ppm': round(co2, 1),
            'temp_c': round(temp, 2),
            'x': round(x, 3),
            'y': round(y, 3),
            'anomaly': self.anomaly,
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
        }

        msg = String()
        msg.data = json.dumps(reading)
        self.pub.publish(msg)
        self.get_logger().info(
            'pH %.2f | CO2 %.0f ppm @ (%.2f, %.2f)%s'
            % (reading['ph'], reading['co2_ppm'], x, y,
               '  <ANOMALY>' if self.anomaly else ''))


def main(args=None):
    rclpy.init(args=args)
    node = PhSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
