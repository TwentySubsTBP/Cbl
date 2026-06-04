#!/usr/bin/env python3
"""Goal-directed movement for the TurtleBot3 digital twin.

Drives the robot to a target (goal_x, goal_y) expressed in the /odom frame using
a simple proportional controller, with reactive LiDAR obstacle avoidance. Stops
within GOAL_TOLERANCE of the target.

Run:
  ros2 run my_tb3_world go_to_goal --ros-args -p goal_x:=1.2 -p goal_y:=0.0

This grows the 'movement script' toward sector/goal navigation (team item 3).
Note: this is a *local* controller - it steers straight at the goal and dodges
obstacles reactively, so it can get stuck in front of walls. For global path
planning around the arena, hand the goal to Nav2 instead (see test_nav.py).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

GOAL_TOLERANCE = 0.15    # m    how close counts as 'arrived'
MAX_LINEAR = 0.15        # m/s  forward speed cap
MAX_ANGULAR = 0.8        # rad/s turn speed cap
HEADING_TOL = 0.15       # rad  turn in place until roughly facing the goal
STOP_DIST = 0.4          # m    front obstacle distance that triggers avoidance
FRONT_ARC_DEG = 25       # deg  half-width of the front cone we watch


def yaw_from_quat(q):
    """Z-axis (yaw) angle from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        # >>> CHANGE THE TARGET LOCATION HERE <<<
        # Goal coordinates (in the /odom frame). Either edit the default
        # numbers below, or override them at run time without touching code:
        #   ros2 run my_tb3_world go_to_goal --ros-args -p goal_x:=1.5 -p goal_y:=0.5
        self.goal_x = float(self.declare_parameter('goal_x', 1.0).value)
        self.goal_y = float(self.declare_parameter('goal_y', 0.0).value)

        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # /odom and /scan are published BEST_EFFORT by the gz bridge.
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pose = None      # (x, y, yaw)
        self.scan = None
        self.create_subscription(Odometry, 'odom', self._on_odom, sensor_qos)
        self.create_subscription(LaserScan, 'scan', self._on_scan, sensor_qos)

        self.reached = False
        self.create_timer(0.1, self._tick)   # 10 Hz control loop
        self.get_logger().info(
            'GoToGoal: driving to (%.2f, %.2f) in the odom frame.'
            % (self.goal_x, self.goal_y))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))

    def _on_scan(self, msg):
        self.scan = msg

    def _front_min(self):
        s = self.scan
        if s is None or not s.ranges:
            return math.inf
        n = len(s.ranges)
        arc = max(1, int(FRONT_ARC_DEG * n / 360))
        idxs = list(range(0, arc)) + list(range(n - arc, n))  # front wraps index 0
        vals = [s.ranges[i] for i in idxs
                if math.isfinite(s.ranges[i]) and s.ranges[i] > 0.0]
        return min(vals) if vals else math.inf

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _publish(self, linear, angular):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(linear)
        m.twist.angular.z = float(angular)
        self.pub.publish(m)

    def _tick(self):
        if self.reached or self.pose is None:
            return
        x, y, yaw = self.pose
        dx, dy = self.goal_x - x, self.goal_y - y
        dist = math.hypot(dx, dy)

        if dist < GOAL_TOLERANCE:
            self._publish(0.0, 0.0)
            self.reached = True
            self.get_logger().info('Goal reached at (%.2f, %.2f).' % (x, y))
            return

        # Reactive obstacle avoidance takes priority over goal seeking.
        if self._front_min() < STOP_DIST:
            self._publish(0.0, MAX_ANGULAR)
            return

        heading_err = norm_angle(math.atan2(dy, dx) - yaw)
        if abs(heading_err) > HEADING_TOL:
            # Not facing the goal yet: turn in place toward it.
            self._publish(0.0, self._clamp(1.5 * heading_err, -MAX_ANGULAR, MAX_ANGULAR))
        else:
            # Facing the goal: drive forward with mild heading correction.
            self._publish(MAX_LINEAR, self._clamp(1.0 * heading_err, -MAX_ANGULAR, MAX_ANGULAR))


def main():
    rclpy.init()
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)   # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
