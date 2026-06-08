#!/usr/bin/env python3
"""Anomaly-zone visualizer: draw the pH leak in the Gazebo scene (NFR3).

This makes the otherwise-invisible water-quality data *visible inside Gazebo*.
It watches /water_quality and, the moment a pH anomaly is flagged, spawns a
translucent red cylinder ("the leak zone") into the digital-twin world at the
robot's reported position. When the anomaly clears, the zone is removed.

This is the visual that backs Act 1 of the demo ("DT highlights zone") and the
NFR3 requirement ("anomaly zones visible in Gazebo").

It is self-contained: it reads /water_quality directly (from ph_sensor), so it
works on its own. The same trigger could later be driven by dt_supervisor's
/mode=ALERT instead, with no change to the spawning logic.

How it draws: Gazebo Sim has no ROS spawn service, so this node calls the gz
transport services /world/<world>/create and /world/<world>/remove via the `gz`
CLI (available in the same container as the sim). Anomalies are edge-triggered
and rare, so the brief synchronous service call is fine.

Run (inside the sim container):
  ros2 run my_tb3_world anomaly_zone_viz
  ros2 run my_tb3_world anomaly_zone_viz --ros-args -p world:=default -p radius:=0.6
"""
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# XML uses single quotes so the whole thing can sit inside a double-quoted
# protobuf-text string field without any escaping.
SDF_TEMPLATE = (
    "<?xml version='1.0'?>"
    "<sdf version='1.8'>"
    "<model name='{name}'>"
    "<static>true</static>"
    "<link name='link'>"
    "<visual name='visual'>"
    "<geometry><cylinder><radius>{radius}</radius><length>0.06</length></cylinder></geometry>"
    "<material>"
    "<ambient>1 0 0 0.6</ambient>"
    "<diffuse>1 0 0 0.6</diffuse>"
    "<specular>1 0 0 0.6</specular>"
    "</material>"
    "</visual>"
    "</link>"
    "</model>"
    "</sdf>"
)


class AnomalyZoneViz(Node):
    def __init__(self):
        super().__init__('anomaly_zone_viz')
        self.world = str(self.declare_parameter('world', 'default').value)
        self.radius = float(self.declare_parameter('radius', 0.5).value)
        self.zone_z = float(self.declare_parameter('zone_z', 0.05).value)
        self.model_name = str(self.declare_parameter('model_name', 'anomaly_zone').value)

        # /water_quality is published BEST_EFFORT by ph_sensor -> match QoS.
        be = QoSProfile(depth=10)
        be.reliability = ReliabilityPolicy.BEST_EFFORT

        self._prev_anomaly = False
        self._spawned = False
        self.create_subscription(String, 'water_quality', self._on_water, be)
        self.get_logger().info(
            "anomaly_zone_viz up: will draw a red leak zone in Gazebo world '%s' "
            "when /water_quality reports an anomaly." % self.world)

    # ----- gz service helpers -----
    def _gz(self, service, reqtype, req):
        """Call a gz transport service via the CLI. Returns True on success."""
        try:
            r = subprocess.run(
                ['gz', 'service', '-s', '/world/%s/%s' % (self.world, service),
                 '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '3000', '--req', req],
                capture_output=True, text=True, timeout=5)
            return 'data: true' in r.stdout
        except (subprocess.SubprocessError, FileNotFoundError) as e:
            self.get_logger().error('gz service call failed: %s' % e)
            return False

    def _remove_zone(self):
        if self._gz('remove', 'gz.msgs.Entity',
                    'name: "%s", type: MODEL' % self.model_name):
            self.get_logger().info('leak zone removed from Gazebo')
        self._spawned = False

    def _spawn_zone(self, x, y):
        # Idempotent: clear any existing zone first so a re-trigger relocates it.
        if self._spawned:
            self._gz('remove', 'gz.msgs.Entity',
                     'name: "%s", type: MODEL' % self.model_name)
        sdf = SDF_TEMPLATE.format(name=self.model_name, radius=self.radius)
        req = ('sdf: "%s", name: "%s", pose: {position: {x: %f, y: %f, z: %f}}'
               % (sdf, self.model_name, x, y, self.zone_z))
        if self._gz('create', 'gz.msgs.EntityFactory', req):
            self._spawned = True
            self.get_logger().warn(
                'LEAK ZONE drawn in Gazebo at (%.2f, %.2f)' % (x, y))
        else:
            self.get_logger().error('failed to spawn leak zone')

    # ----- telemetry -----
    def _on_water(self, msg):
        import json
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        anomaly = bool(data.get('anomaly'))
        if anomaly and not self._prev_anomaly:
            self._spawn_zone(float(data.get('x', 0.0)), float(data.get('y', 0.0)))
        elif not anomaly and self._prev_anomaly:
            self._remove_zone()
        self._prev_anomaly = anomaly


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyZoneViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up the zone on shutdown so it doesn't linger in the world.
        if node._spawned:
            node._remove_zone()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
