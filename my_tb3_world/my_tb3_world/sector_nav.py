#!/usr/bin/env python3
"""Sector navigation: patrol the arena sector-by-sector (team items 3.1/3.2).

Divides the arena into a grid of sectors and drives the robot to each sector
centre in a snake/boustrophedon order for efficient, systematic coverage -- the
'survey the whole ocean for water quality' behaviour. It publishes the current
sector centre as /goal_pose (consumed by go_to_goal) and reports progress on
/sector. When a sector is reached it advances to the next, looping forever
(continuous patrol).

This turns the one-shot go_to_goal into a multi-waypoint coverage mission.

Run:
  ros2 run my_tb3_world sector_nav
  ros2 run my_tb3_world sector_nav --ros-args -p rows:=2 -p cols:=3 \
      -p x_min:=-1.0 -p x_max:=2.0 -p y_min:=-1.5 -p y_max:=1.5
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class SectorNav(Node):
    def __init__(self):
        super().__init__('sector_nav')
        # Arena extent (odom frame) and grid resolution. Defaults suit the demo
        # arena; tune to the real map bounds.
        x_min = float(self.declare_parameter('x_min', -1.0).value)
        x_max = float(self.declare_parameter('x_max', 2.0).value)
        y_min = float(self.declare_parameter('y_min', -1.5).value)
        y_max = float(self.declare_parameter('y_max', 1.5).value)
        self.rows = int(self.declare_parameter('rows', 2).value)
        self.cols = int(self.declare_parameter('cols', 2).value)
        self.tol = float(self.declare_parameter('arrive_tol', 0.35).value)
        self.loop = bool(self.declare_parameter('loop', True).value)

        self.sectors = self._build_sectors(x_min, x_max, y_min, y_max)
        self.idx = 0
        self.pose = None
        self._visited = 0

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, 'odom', self._on_odom, sensor_qos)

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', latched)
        self.status_pub = self.create_publisher(String, 'sector', 10)

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            'sector_nav up: %d sectors (%dx%d grid), snake order, arrive_tol %.2fm.'
            % (len(self.sectors), self.rows, self.cols, self.tol))
        self._publish_goal()  # send the first sector immediately

    def _build_sectors(self, x_min, x_max, y_min, y_max):
        """Sector centres in snake order (left-right, then right-left, ...)."""
        cells = []
        for r in range(self.rows):
            cy = y_min + (r + 0.5) * (y_max - y_min) / self.rows
            cols = range(self.cols) if r % 2 == 0 else range(self.cols - 1, -1, -1)
            for c in cols:
                cx = x_min + (c + 0.5) * (x_max - x_min) / self.cols
                cells.append((round(cx, 3), round(cy, 3)))
        return cells

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _publish_goal(self):
        cx, cy = self.sectors[self.idx]
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom'
        g.pose.position.x = float(cx)
        g.pose.position.y = float(cy)
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    def _tick(self):
        cx, cy = self.sectors[self.idx]
        reached = False
        if self.pose is not None:
            dx, dy = cx - self.pose[0], cy - self.pose[1]
            reached = (dx * dx + dy * dy) ** 0.5 < self.tol

        if reached:
            self._visited += 1
            self.get_logger().info('reached sector %d at (%.2f, %.2f)'
                                   % (self.idx, cx, cy))
            nxt = self.idx + 1
            if nxt >= len(self.sectors):
                if not self.loop:
                    self.get_logger().info('patrol complete (%d sectors).' % len(self.sectors))
                    return
                nxt = 0
            self.idx = nxt
            self._publish_goal()

        status = {
            'sector': self.idx,
            'target': list(self.sectors[self.idx]),
            'total': len(self.sectors),
            'visited': self._visited,
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = SectorNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
