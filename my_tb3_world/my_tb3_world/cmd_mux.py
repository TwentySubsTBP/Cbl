#!/usr/bin/env python3
"""Velocity command mux: arbitrate /cmd_vel by priority (integration glue).

Several nodes want to drive the robot, and without arbitration they all publish
to /cmd_vel at once and their messages interleave (the robot twitches). This mux
is the single writer of /cmd_vel. It listens to three prioritised input topics
and forwards the highest-priority one that is currently "fresh":

    1. HALT     (cmd_vel_halt)     -- comms_watchdog: lost-twin safety stop   [highest]
    2. OVERRIDE (cmd_vel_override) -- dt_supervisor: obstacle/safety override
    3. NAV      (cmd_vel_nav)      -- go_to_goal / mover: normal navigation    [lowest]

A source counts as fresh if it published within `timeout` seconds. If nothing is
fresh, the mux publishes a zero command (fail-safe: a silent system stops).

This is the arbitration layer for the integration launch (team item 11); it
fixes the multi-publisher /cmd_vel contention found in the system dry-run.

Run (normally started by waverider.launch.py):
  ros2 run my_tb3_world cmd_mux
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')
        halt_topic = str(self.declare_parameter('halt_topic', 'cmd_vel_halt').value)
        override_topic = str(self.declare_parameter('override_topic', 'cmd_vel_override').value)
        nav_topic = str(self.declare_parameter('nav_topic', 'cmd_vel_nav').value)
        output_topic = str(self.declare_parameter('output_topic', 'cmd_vel').value)
        self.timeout = float(self.declare_parameter('timeout', 0.4).value)
        rate = float(self.declare_parameter('rate', 20.0).value)

        # Priority order: first match wins. (name, topic, state slot)
        self._sources = ['HALT', 'OVERRIDE', 'NAV']
        self._last_msg = {s: None for s in self._sources}
        self._last_t = {s: 0.0 for s in self._sources}
        self._active = None  # currently forwarded source (for change logging)

        self.create_subscription(TwistStamped, halt_topic,
                                 lambda m: self._on_cmd('HALT', m), 10)
        self.create_subscription(TwistStamped, override_topic,
                                 lambda m: self._on_cmd('OVERRIDE', m), 10)
        self.create_subscription(TwistStamped, nav_topic,
                                 lambda m: self._on_cmd('NAV', m), 10)

        self.pub = self.create_publisher(TwistStamped, output_topic, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'cmd_mux up: HALT(%s) > OVERRIDE(%s) > NAV(%s) -> /%s, freshness %.2fs.'
            % (halt_topic, override_topic, nav_topic, output_topic, self.timeout))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, source, msg):
        self._last_msg[source] = msg
        self._last_t[source] = self._now()

    def _stop_msg(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        return m

    def _tick(self):
        now = self._now()
        chosen, out = None, None
        for s in self._sources:                 # highest priority first
            if self._last_msg[s] is not None and (now - self._last_t[s]) < self.timeout:
                chosen, out = s, self._last_msg[s]
                break
        if out is None:                          # nothing fresh -> fail-safe stop
            chosen, out = 'NONE(stop)', self._stop_msg()

        out.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(out)

        if chosen != self._active:
            self.get_logger().info('cmd source -> %s' % chosen)
            self._active = chosen


def main(args=None):
    rclpy.init(args=args)
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
