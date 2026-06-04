#!/usr/bin/env python3
"""Reactive movement node for the TurtleBot3 digital twin.

Publishes geometry_msgs/TwistStamped to /cmd_vel (the type modern Gazebo / the
ros_gz bridge expects on Jazzy) and reads the LiDAR on /scan. Behaviour:
drive forward; when an obstacle is detected within STOP_DIST in the front arc,
turn in place until the path ahead is clear.

This is the base 'movement script' (team to-do items 1 & 3): it already fuses
sensor data (/scan) with motion (/cmd_vel). Extend tick() to set sector-based
navigation goals instead of simple forward/turn.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

FORWARD_SPEED = 0.15     # m/s   forward velocity when path is clear
TURN_SPEED = 0.5         # rad/s rotation velocity when avoiding
STOP_DIST = 0.5          # m     obstacle distance that triggers a turn
FRONT_ARC_DEG = 30       # deg   half-width of the front cone we watch


class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # LiDAR publishes BEST_EFFORT; a RELIABLE subscriber receives nothing.
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.scan = None
        self.create_subscription(LaserScan, 'scan', self._on_scan, scan_qos)

        self.create_timer(0.1, self._tick)  # 10 Hz control loop
        self.get_logger().info(
            'Mover started: forward until obstacle < %.2fm, then turn.' % STOP_DIST)

    def _on_scan(self, msg):
        self.scan = msg

    def _front_min(self):
        """Smallest valid range within +/- FRONT_ARC_DEG of straight ahead."""
        s = self.scan
        if s is None or not s.ranges:
            return math.inf
        n = len(s.ranges)
        arc = max(1, int(FRONT_ARC_DEG * n / 360))
        idxs = list(range(0, arc)) + list(range(n - arc, n))  # front wraps index 0
        vals = [s.ranges[i] for i in idxs
                if math.isfinite(s.ranges[i]) and s.ranges[i] > 0.0]
        return min(vals) if vals else math.inf

    def _publish(self, linear, angular):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(linear)
        m.twist.angular.z = float(angular)
        self.pub.publish(m)

    def _tick(self):
        if self._front_min() < STOP_DIST:
            self._publish(0.0, TURN_SPEED)      # obstacle ahead -> turn in place
        else:
            self._publish(FORWARD_SPEED, 0.0)   # clear -> drive forward


def main():
    rclpy.init()
    node = Mover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)  # stop the robot on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
