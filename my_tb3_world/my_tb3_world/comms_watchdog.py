#!/usr/bin/env python3
"""Comms watchdog: halt the robot if it loses the digital twin (NFR2).

Safety requirement: if the physical robot stops hearing from the twin for more
than ~5 s (lost link, twin crashed, Wi-Fi cut), it must stop moving rather than
keep driving blind. This node watches the twin's heartbeat and, when it goes
stale, repeatedly publishes a zero /cmd_vel to hold the robot still until the
twin comes back.

Heartbeat source: dt_supervisor publishes /mode continuously (~5 Hz), so that
doubles as the twin's "I'm alive" signal. Any steady twin->robot topic works;
override with -p heartbeat_topic:=<name>.

Behaviour:
  - waits for the first heartbeat before arming (so it doesn't fight a system
    that simply hasn't started yet); set require_first_heartbeat:=false to arm
    immediately and fail safe from boot.
  - heartbeat stale > timeout  -> COMMS LOST: spam stop on /cmd_vel, /twin_alive=false
  - heartbeat returns          -> COMMS OK: release control, /twin_alive=true

Demo (Act 3): run the full stack, then kill dt_supervisor (or cut the link) and
the robot halts within `timeout` seconds.

Note: if the team's twin_safety_node / tb3_safety_stop ever lands in the repo,
reconcile with it -- it may cover this same NFR2 halt-on-comms-loss.

Run:
  ros2 run my_tb3_world comms_watchdog
  ros2 run my_tb3_world comms_watchdog --ros-args -p timeout:=5.0 -p heartbeat_topic:=mode
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TwistStamped


class CommsWatchdog(Node):
    def __init__(self):
        super().__init__('comms_watchdog')
        self.timeout = float(self.declare_parameter('timeout', 5.0).value)
        self.heartbeat_topic = str(self.declare_parameter('heartbeat_topic', 'mode').value)
        self.check_rate = float(self.declare_parameter('check_rate', 10.0).value)
        self.require_first = bool(self.declare_parameter('require_first_heartbeat', True).value)

        self._last_hb = None      # time (sec) of last heartbeat
        self._lost = False        # currently in COMMS LOST state?

        self.create_subscription(String, self.heartbeat_topic, self._on_heartbeat, 10)
        # Halt topic is configurable so the integration launch routes it to the
        # cmd_mux highest-priority input (cmd_vel_halt); defaults to /cmd_vel.
        cmd_topic = str(self.declare_parameter('cmd_vel_topic', 'cmd_vel').value)
        self.cmd_pub = self.create_publisher(TwistStamped, cmd_topic, 10)
        self.alive_pub = self.create_publisher(Bool, 'twin_alive', 10)

        self.create_timer(1.0 / self.check_rate, self._tick)
        self.get_logger().info(
            "comms_watchdog up: halting the robot if '/%s' is silent for >%.1fs."
            % (self.heartbeat_topic, self.timeout))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_heartbeat(self, _msg):
        self._last_hb = self._now()
        if self._lost:
            self._lost = False
            self.get_logger().info('COMMS OK: twin heartbeat restored, releasing hold.')

    def _publish_stop(self):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = 0.0
        m.twist.angular.z = 0.0
        self.cmd_pub.publish(m)

    def _tick(self):
        # Not armed yet: never heard the twin and we're configured to wait.
        if self._last_hb is None:
            if self.require_first:
                self.alive_pub.publish(Bool(data=False))
                return
            stale = True
        else:
            stale = (self._now() - self._last_hb) > self.timeout

        self.alive_pub.publish(Bool(data=not stale))

        if stale:
            if not self._lost:
                self._lost = True
                self.get_logger().warn(
                    'COMMS LOST: no twin heartbeat for >%.1fs -- HALTING robot.'
                    % self.timeout)
            self._publish_stop()   # keep asserting stop while the link is down


def main(args=None):
    rclpy.init(args=args)
    node = CommsWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
