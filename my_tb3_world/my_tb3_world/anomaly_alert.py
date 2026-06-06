#!/usr/bin/env python3
"""Anomaly alert: detect a sudden pH drop and emit a structured alert (FR4).

Automated alerting for the digital twin. It watches /water_quality and raises a
structured alert when the water turns bad -- either a sudden pH *drop* (rate of
change) or pH falling below a critical level (a sustained acidification / leak).
Each alert is:
  - published as JSON on /alerts (for other systems / a dashboard), and
  - logged to the console AND appended to a log file as evidence for the
    report/video.

Alerts are edge-triggered (one per event) with hysteresis, so a single leak
produces one RAISED alert and one CLEARED alert -- not a flood.

This complements dt_supervisor: the supervisor *reacts* (mode/goal), this node
*reports* (the FR4 alert payload + audit log).

Run:
  ros2 run my_tb3_world anomaly_alert
  ros2 run my_tb3_world anomaly_alert --ros-args -p ph_critical:=7.0 -p drop_rate:=0.5
"""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class AnomalyAlert(Node):
    def __init__(self):
        super().__init__('anomaly_alert')
        # pH at/under this is a problem; recovery needs ph_critical + hysteresis.
        self.ph_critical = float(self.declare_parameter('ph_critical', 7.0).value)
        self.hysteresis = float(self.declare_parameter('hysteresis', 0.3).value)
        # A drop larger than this between consecutive readings is itself an alert
        # (catches the *sudden* drop even before the level is breached).
        self.drop_rate = float(self.declare_parameter('drop_rate', 0.5).value)
        self.log_file = str(self.declare_parameter(
            'log_file', '/tmp/waverider_alerts.log').value)

        be = QoSProfile(depth=10)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(String, 'water_quality', self._on_water, be)
        # Alerts are important -> reliable QoS so a dashboard never misses one.
        self.pub = self.create_publisher(String, 'alerts', 10)

        self._prev_ph = None
        self._alerting = False
        self.get_logger().info(
            'anomaly_alert up: raising /alerts when pH drops >%.2f/reading or '
            'falls below %.2f. Log: %s' % (self.drop_rate, self.ph_critical, self.log_file))

    def _on_water(self, msg):
        try:
            d = json.loads(msg.data)
            ph = float(d['ph'])
        except (ValueError, TypeError, KeyError):
            return

        drop = (self._prev_ph - ph) if self._prev_ph is not None else 0.0
        self._prev_ph = ph

        breached = ph <= self.ph_critical
        sudden = drop >= self.drop_rate

        if not self._alerting and (breached or sudden):
            self._alerting = True
            reason = ('sudden pH drop of %.2f' % drop) if sudden else \
                     ('pH %.2f below critical %.2f' % (ph, self.ph_critical))
            self._emit('RAISED', 'WARNING', d, ph, reason)
        elif self._alerting and ph >= self.ph_critical + self.hysteresis:
            self._alerting = False
            self._emit('CLEARED', 'INFO', d, ph, 'pH recovered to %.2f' % ph)

    def _emit(self, state, severity, d, ph, reason):
        alert = {
            'state': state,           # RAISED | CLEARED
            'severity': severity,     # WARNING | INFO
            'type': 'water_quality',
            'ph': round(ph, 3),
            'co2_ppm': d.get('co2_ppm'),
            'x': d.get('x'),
            'y': d.get('y'),
            'reason': reason,
            'stamp': self.get_clock().now().nanoseconds * 1e-9,
        }
        line = json.dumps(alert)
        self.pub.publish(String(data=line))
        if state == 'RAISED':
            self.get_logger().warn('ALERT %s: %s at (%s, %s)'
                                   % (severity, reason, alert['x'], alert['y']))
        else:
            self.get_logger().info('ALERT %s: %s' % (state, reason))
        # Append to the evidence log (best-effort; never crash the node on IO).
        try:
            with open(self.log_file, 'a') as f:
                f.write(line + '\n')
        except OSError as e:
            self.get_logger().error('could not write alert log: %s' % e)


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyAlert()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
