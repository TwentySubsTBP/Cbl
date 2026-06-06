#!/usr/bin/env python3
"""Digital-twin supervisor: the bidirectional hub of the WaveRider twin (FR2.1/FR2.2).

This is the "brain" of the digital twin. It replaces the old one-way velocity
bridge (resource/test_odom.py) with a two-way hub:

  robot -> twin (telemetry IN):
    /water_quality  (std_msgs/String JSON, from ph_sensor)
    /obstacle_info  (std_msgs/String JSON, from obstacle_info)
    /scan           (sensor_msgs/LaserScan, raw LiDAR)
    /odom           (nav_msgs/Odometry, pose)

  twin -> robot (commands OUT):
    /mode       (std_msgs/String JSON: current decision + reason + health)
    /goal_pose  (geometry_msgs/PoseStamped: where the twin wants the robot to go)
    /cmd_vel    (geometry_msgs/TwistStamped: safety override, e.g. emergency stop)

It runs a small priority state machine each tick:
    AVOID  (obstacle within critical distance)  -> stop override on /cmd_vel
    ALERT  (pH anomaly detected)                -> /goal_pose at the leak to investigate
    NORMAL (nominal)                            -> hands off, let the local controller drive

Run:
  ros2 run my_tb3_world dt_supervisor
  ros2 run my_tb3_world dt_supervisor --ros-args -p critical_dist:=0.3 -p safety_override:=false

Scope note: this hub operates on a single ROS domain (the twin/sim domain), which
is what the all-sim demo needs. The physical<->twin *network* transport across ROS
domains (what test_odom.py started, Domain 30 <-> Domain 0) is a separate
integration concern and can be bridged in front of this node later.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

STALE_SEC = 3.0   # telemetry older than this is treated as "comms not healthy"


class DtSupervisor(Node):
    def __init__(self):
        super().__init__('dt_supervisor')

        # --- Tunables ---
        self.critical_dist = float(self.declare_parameter('critical_dist', 0.25).value)
        # When True, the twin may publish a stop on /cmd_vel to override the robot.
        self.safety_override = bool(self.declare_parameter('safety_override', True).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 5.0).value)

        # /scan, /odom, and the JSON telemetry topics are published BEST_EFFORT.
        be = QoSProfile(depth=10)
        be.reliability = ReliabilityPolicy.BEST_EFFORT

        # --- Twin state (latest telemetry) ---
        self.water = None        # parsed /water_quality dict
        self.obstacle = None     # parsed /obstacle_info dict
        self.scan_min = math.inf
        self.pose = None         # (x, y)
        self._t_water = None     # rx time of last water_quality (sec)
        self._t_obstacle = None
        self._prev_anomaly = False

        # robot -> twin (telemetry IN)
        self.create_subscription(String, 'water_quality', self._on_water, be)
        self.create_subscription(String, 'obstacle_info', self._on_obstacle, be)
        self.create_subscription(LaserScan, 'scan', self._on_scan, be)
        self.create_subscription(Odometry, 'odom', self._on_odom, be)

        # twin -> robot (commands OUT)
        self.mode_pub = self.create_publisher(String, 'mode', 10)
        # Latch the goal: it is published once per anomaly (edge-triggered), so a
        # consumer (Nav2 / go_to_goal) that subscribes later must still receive it.
        goal_qos = QoSProfile(depth=1)
        goal_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', goal_qos)
        # Override topic is configurable so the integration launch routes it to
        # the cmd_mux safety input (cmd_vel_override); defaults to /cmd_vel.
        cmd_topic = str(self.declare_parameter('cmd_vel_topic', 'cmd_vel').value)
        self.cmd_pub = self.create_publisher(TwistStamped, cmd_topic, 10)

        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            'dt_supervisor up: telemetry IN (/water_quality,/obstacle_info,/scan,/odom) '
            '-> decisions OUT (/mode,/goal_pose,/cmd_vel). '
            'critical_dist=%.2fm, safety_override=%s.'
            % (self.critical_dist, self.safety_override))

    # ----- telemetry callbacks -----
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_water(self, msg):
        try:
            self.water = json.loads(msg.data)
            self._t_water = self._now()
        except (ValueError, TypeError):
            self.get_logger().warn('could not parse /water_quality JSON')

    def _on_obstacle(self, msg):
        try:
            self.obstacle = json.loads(msg.data)
            self._t_obstacle = self._now()
        except (ValueError, TypeError):
            self.get_logger().warn('could not parse /obstacle_info JSON')

    def _on_scan(self, msg):
        vals = [r for r in msg.ranges if math.isfinite(r) and r > 0.0]
        self.scan_min = min(vals) if vals else math.inf

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    # ----- helpers -----
    def _front_dist(self):
        """Closest obstacle in front: prefer obstacle_info, fall back to raw scan."""
        if self.obstacle is not None:
            mf = self.obstacle.get('min_front')
            if isinstance(mf, (int, float)):
                return float(mf)
        return self.scan_min

    def _comms_ok(self):
        now = self._now()
        fresh = (self._t_water is not None and now - self._t_water < STALE_SEC)
        return fresh

    def _publish_stop(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = 0.0
        m.twist.angular.z = 0.0
        self.cmd_pub.publish(m)

    def _send_goal(self, x, y):
        g = PoseStamped()
        g.header.stamp = self.get_clock().now().to_msg()
        g.header.frame_id = 'odom'
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

    # ----- decision loop -----
    def _tick(self):
        front = self._front_dist()
        anomaly = bool(self.water.get('anomaly')) if self.water else False
        comms_ok = self._comms_ok()

        # Anomaly response (emit investigate-goal + alert) is decoupled from the
        # motion-safety priority below: a leak that happens while the robot is
        # dodging an obstacle must still produce the goal, not be swallowed by
        # AVOID. Edge-triggered so it fires once per anomaly episode.
        if anomaly and not self._prev_anomaly and self.water is not None:
            x = self.water.get('x', 0.0)
            y = self.water.get('y', 0.0)
            self._send_goal(x, y)
            self.get_logger().warn(
                'ALERT: pH %.2f anomaly at (%.2f, %.2f) -> /goal_pose to investigate'
                % (self.water.get('ph', float('nan')), x, y))
        self._prev_anomaly = anomaly

        # Motion-safety priority drives /cmd_vel and the reported mode:
        # AVOID (stop override) outranks ALERT outranks NORMAL.
        if front < self.critical_dist:
            mode, reason = 'AVOID', 'obstacle %.2fm < %.2fm' % (front, self.critical_dist)
            if self.safety_override:
                self._publish_stop()
        elif anomaly:
            mode, reason = 'ALERT', 'pH anomaly detected'
        else:
            mode, reason = 'NORMAL', 'nominal'

        status = {
            'mode': mode,
            'reason': reason,
            'comms_ok': comms_ok,
            'ph': self.water.get('ph') if self.water else None,
            'front_m': round(front, 2) if math.isfinite(front) else None,
            'stamp': self._now(),
            # Echo the source reading's timestamp so latency_logger can measure
            # the full robot->twin->response round-trip (NFR1).
            'src_stamp': self.water.get('stamp') if self.water else None,
        }
        out = String()
        out.data = json.dumps(status)
        self.mode_pub.publish(out)
        # Log only on mode changes or non-normal modes to keep the console readable.
        if mode != 'NORMAL':
            self.get_logger().info('mode=%s (%s)' % (mode, reason))


def main(args=None):
    rclpy.init(args=args)
    node = DtSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
