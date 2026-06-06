#!/usr/bin/env python3
"""Latency logger: measure robot<->twin round-trip delay vs NFR1 (<1000ms).

Round-trip path measured:
    ph_sensor (robot) stamps a /water_quality reading at T0
      -> dt_supervisor (twin) consumes it and publishes /mode, echoing that T0
         back as 'src_stamp'
      -> this node (robot side) receives /mode at T1
    round_trip = T1 - T0

For every /mode message carrying a src_stamp it computes the latency, publishes
it on /latency_ms (std_msgs/Float64, easy to rosbag), tracks running stats, and
flags any sample over the NFR1 limit. A periodic summary is logged and appended
to an evidence file for the report/video.

Capture evidence:
  ros2 bag record /latency_ms /water_quality /mode /alerts
  # then run the system + trigger events; the bag is your NFR1 proof.

Run:
  ros2 run my_tb3_world latency_logger
  ros2 run my_tb3_world latency_logger --ros-args -p limit_ms:=1000.0
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64


class LatencyLogger(Node):
    def __init__(self):
        super().__init__('latency_logger')
        self.limit_ms = float(self.declare_parameter('limit_ms', 1000.0).value)
        self.source_topic = str(self.declare_parameter('source_topic', 'mode').value)
        self.log_file = str(self.declare_parameter(
            'log_file', '/tmp/waverider_latency.log').value)
        self.summary_period = float(self.declare_parameter('summary_period', 5.0).value)

        self.create_subscription(String, self.source_topic, self._on_mode, 10)
        self.pub = self.create_publisher(Float64, 'latency_ms', 10)

        self._n = 0
        self._sum = 0.0
        self._min = float('inf')
        self._max = 0.0
        self._over = 0          # samples exceeding the NFR1 limit
        self.create_timer(self.summary_period, self._summary)
        self.get_logger().info(
            'latency_logger up: measuring robot<->twin round-trip on /%s vs NFR1 '
            'limit %.0f ms. Publishing /latency_ms; log %s'
            % (self.source_topic, self.limit_ms, self.log_file))

    def _on_mode(self, msg):
        try:
            d = json.loads(msg.data)
            src = d.get('src_stamp')
        except (ValueError, TypeError):
            return
        if src is None:
            return  # twin had no reading to act on yet
        now = self.get_clock().now().nanoseconds * 1e-9
        latency_ms = (now - float(src)) * 1000.0
        if latency_ms < 0:
            return  # clock skew / stale; ignore

        self.pub.publish(Float64(data=latency_ms))
        self._n += 1
        self._sum += latency_ms
        self._min = min(self._min, latency_ms)
        self._max = max(self._max, latency_ms)
        if latency_ms > self.limit_ms:
            self._over += 1
            self.get_logger().warn(
                'NFR1 BREACH: round-trip %.0f ms > %.0f ms' % (latency_ms, self.limit_ms))

    def _summary(self):
        if self._n == 0:
            return
        avg = self._sum / self._n
        ok = 'PASS' if self._max <= self.limit_ms else 'FAIL'
        line = ('latency ms over %d samples: avg=%.1f min=%.1f max=%.1f '
                'over_limit=%d NFR1(%.0fms)=%s'
                % (self._n, avg, self._min, self._max, self._over, self.limit_ms, ok))
        self.get_logger().info(line)
        try:
            with open(self.log_file, 'a') as f:
                f.write(line + '\n')
        except OSError as e:
            self.get_logger().error('could not write latency log: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = LatencyLogger()
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
