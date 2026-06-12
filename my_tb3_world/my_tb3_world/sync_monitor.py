#!/usr/bin/env python3
"""Sync monitor: measure and log the twin's state-sync error (course Week-6 deck).

The course defines the gap between the entity's actual state and the state the
twin currently believes as THE sync error a digital twin must measure and log
(Week 6 slides 14/15: "log state on both sides simultaneously; the difference
is your measured sync error"). The exemplar twin supervisor (slide 39) also
exposes a /sync_status output next to /mode and /alerts.

This node provides both:
  - /sync_error_m  (std_msgs/Float64) distance in metres between the robot's
        current /odom position and the position in the twin's last consumed
        water-quality reading -- i.e. how far the robot has moved beyond what
        the twin currently knows. Easy to rosbag and plot.
  - /sync_status   (std_msgs/String JSON) the twin-side health summary:
        sync error, age of the twin's state, sync_ok flag, robot + twin poses.

A periodic summary (avg/max error, stale count) is logged and appended to an
evidence file, mirroring latency_logger (which covers the *time* dimension of
sync; this covers the *state* dimension).

Run:
  ros2 run my_tb3_world sync_monitor
  ros2 run my_tb3_world sync_monitor --ros-args -p error_limit:=0.3 -p stale_sec:=2.0
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry


class SyncMonitor(Node):
    def __init__(self):
        super().__init__('sync_monitor')
        # Above this positional gap the twin is considered out of sync. With a
        # 2 Hz sensor and 0.15 m/s drive speed the expected gap is <= ~0.08 m,
        # so 0.3 m flags genuine desync (stalled telemetry), not normal lag.
        self.error_limit = float(self.declare_parameter('error_limit', 0.3).value)
        # Twin state older than this is stale regardless of position.
        self.stale_sec = float(self.declare_parameter('stale_sec', 2.0).value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 2.0).value)
        self.log_file = str(self.declare_parameter(
            'log_file', '/tmp/waverider_sync.log').value)
        self.summary_period = float(self.declare_parameter('summary_period', 5.0).value)

        be = QoSProfile(depth=10)
        be.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pose = None        # robot's actual position, from /odom
        self.twin_view = None   # twin's last consumed reading: (x, y, stamp)
        self.create_subscription(Odometry, 'odom', self._on_odom, be)
        self.create_subscription(String, 'water_quality', self._on_water, be)

        # Status is twin-critical state -> reliable, like /mode and /alerts.
        self.status_pub = self.create_publisher(String, 'sync_status', 10)
        self.error_pub = self.create_publisher(Float64, 'sync_error_m', 10)

        self._n = 0
        self._sum = 0.0
        self._max = 0.0
        self._desync = 0      # samples breaching error_limit or stale_sec
        self._was_ok = True
        self.create_timer(1.0 / self.rate_hz, self._tick)
        self.create_timer(self.summary_period, self._summary)
        self.get_logger().info(
            'sync_monitor up: /sync_error_m + /sync_status, limits %.2fm / %.1fs '
            'stale. Log: %s' % (self.error_limit, self.stale_sec, self.log_file))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _on_water(self, msg):
        try:
            d = json.loads(msg.data)
            self.twin_view = (float(d['x']), float(d['y']), float(d['stamp']))
        except (ValueError, TypeError, KeyError):
            pass

    def _tick(self):
        if self.pose is None or self.twin_view is None:
            return  # nothing to compare yet
        rx, ry = self.pose
        tx, ty, tstamp = self.twin_view
        err = math.hypot(rx - tx, ry - ty)
        age = self._now() - tstamp
        ok = err <= self.error_limit and age <= self.stale_sec

        self.error_pub.publish(Float64(data=err))
        status = {
            'sync_ok': ok,
            'sync_error_m': round(err, 4),
            'state_age_s': round(age, 3),
            'robot_xy': [round(rx, 3), round(ry, 3)],
            'twin_xy': [round(tx, 3), round(ty, 3)],
            'stamp': self._now(),
        }
        self.status_pub.publish(String(data=json.dumps(status)))

        self._n += 1
        self._sum += err
        self._max = max(self._max, err)
        if not ok:
            self._desync += 1
        # Log only on health transitions to keep the console readable.
        if ok != self._was_ok:
            if ok:
                self.get_logger().info('SYNC OK: error %.3fm, state age %.2fs' % (err, age))
            else:
                self.get_logger().warn(
                    'TWIN DESYNC: error %.3fm (limit %.2f), state age %.2fs (limit %.1f)'
                    % (err, self.error_limit, age, self.stale_sec))
            self._was_ok = ok

    def _summary(self):
        if self._n == 0:
            return
        line = ('sync error m over %d samples: avg=%.4f max=%.4f desync=%d '
                'limit(%.2fm/%.1fs)=%s'
                % (self._n, self._sum / self._n, self._max, self._desync,
                   self.error_limit, self.stale_sec,
                   'PASS' if self._desync == 0 else 'CHECK'))
        self.get_logger().info(line)
        try:
            with open(self.log_file, 'a') as f:
                f.write(line + '\n')
        except OSError as e:
            self.get_logger().error('could not write sync log: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = SyncMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._summary()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
