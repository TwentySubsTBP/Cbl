#!/usr/bin/env python3
"""Hazard manager: spawn DT-only currents/storms and broadcast them (FR3, env interaction).

This is the digital twin's *environment*. It injects a hazard (a current/storm
region) that exists ONLY in the twin -- it has no physical collision, the robot
never bumps it -- yet the robot must still react to it. That is the whole point
of "environmental interaction": the twin shapes the world and the physical robot
changes its behaviour in response.

It does two things when a hazard is active:
  1. draws a translucent BLUE zone in the Gazebo scene (so it's visible, NFR3),
  2. publishes the hazard on /hazard_zone (latched JSON) so go_to_goal can
     re-route around it.

Trigger it like the pH leak:
  ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: true}"   # storm ON
  ros2 topic pub --once /spawn_hazard std_msgs/Bool "{data: false}"  # storm OFF

Run (inside the sim container):
  ros2 run my_tb3_world hazard_manager --ros-args -p hazard_x:=0.0 -p hazard_y:=0.0 -p radius:=0.6
"""
import json
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool

# Single-quoted XML so it can sit inside a double-quoted protobuf string field.
SDF_TEMPLATE = (
    "<?xml version='1.0'?>"
    "<sdf version='1.8'>"
    "<model name='{name}'>"
    "<static>true</static>"
    "<link name='link'>"
    "<visual name='visual'>"
    "<geometry><cylinder><radius>{radius}</radius><length>0.08</length></cylinder></geometry>"
    "<material>"
    "<ambient>0 0.3 1 0.5</ambient>"
    "<diffuse>0 0.3 1 0.5</diffuse>"
    "<specular>0 0.3 1 0.5</specular>"
    "</material>"
    "</visual>"
    "</link>"
    "</model>"
    "</sdf>"
)


class HazardManager(Node):
    def __init__(self):
        super().__init__('hazard_manager')
        self.world = str(self.declare_parameter('world', 'default').value)
        self.hazard_x = float(self.declare_parameter('hazard_x', 0.0).value)
        self.hazard_y = float(self.declare_parameter('hazard_y', 0.0).value)
        self.radius = float(self.declare_parameter('radius', 0.6).value)
        self.kind = str(self.declare_parameter('kind', 'current').value)
        self.model_name = str(self.declare_parameter('model_name', 'hazard_zone').value)

        # Latch /hazard_zone so go_to_goal gets the current hazard even if it
        # subscribes after the hazard was spawned.
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.zone_pub = self.create_publisher(String, 'hazard_zone', latched)

        self._active = False
        self._spawned = False
        self.create_subscription(Bool, 'spawn_hazard', self._on_trigger, 10)
        # Re-publish state at 2 Hz so late subscribers / lost samples self-heal.
        self.create_timer(0.5, self._publish_state)
        self._publish_state()  # announce "no hazard" at startup
        self.get_logger().info(
            "hazard_manager up: toggle /spawn_hazard to inject a DT-only '%s' at "
            "(%.2f, %.2f) r=%.2f." % (self.kind, self.hazard_x, self.hazard_y, self.radius))

    # ----- gz spawn helpers -----
    def _gz(self, service, reqtype, req):
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

    def _spawn(self):
        if self._spawned:
            return
        sdf = SDF_TEMPLATE.format(name=self.model_name, radius=self.radius)
        req = ('sdf: "%s", name: "%s", pose: {position: {x: %f, y: %f, z: 0.06}}'
               % (sdf, self.model_name, self.hazard_x, self.hazard_y))
        if self._gz('create', 'gz.msgs.EntityFactory', req):
            self._spawned = True
            self.get_logger().warn(
                'HAZARD (%s) spawned in DT at (%.2f, %.2f)'
                % (self.kind, self.hazard_x, self.hazard_y))

    def _despawn(self):
        if not self._spawned:
            return
        if self._gz('remove', 'gz.msgs.Entity',
                    'name: "%s", type: MODEL' % self.model_name):
            self.get_logger().info('hazard removed from DT')
        self._spawned = False

    # ----- triggers / state -----
    def _on_trigger(self, msg):
        active = bool(msg.data)
        if active and not self._active:
            self._spawn()
        elif not active and self._active:
            self._despawn()
        self._active = active
        self._publish_state()

    def _publish_state(self):
        state = {
            'active': self._active,
            'kind': self.kind,
            'x': self.hazard_x,
            'y': self.hazard_y,
            'radius': self.radius,
        }
        m = String()
        m.data = json.dumps(state)
        self.zone_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = HazardManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._spawned:
            node._despawn()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
