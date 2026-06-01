#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist

class VelocitySyncBridge:
    def __init__(self):
        # 1. Initialize Domain 30 (Physical Robot Monitor)
        self.phys_context = rclpy.Context()
        rclpy.init(context=self.phys_context, domain_id=30)
        self.phys_node = Node('physical_velocity_listener', context=self.phys_context)
        
        # Assign explicit executor for Domain 30
        self.phys_executor = SingleThreadedExecutor(context=self.phys_context)
        self.phys_executor.add_node(self.phys_node)
        
        # Subscribe to the velocity commands driving the physical robot
        self.phys_node.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        # 2. Initialize Domain 0 (Digital Robot Shadow Mirror)
        self.sim_context = rclpy.Context()
        rclpy.init(context=self.sim_context, domain_id=0)
        self.sim_node = Node('digital_velocity_mirror', context=self.sim_context)
        
        # Assign explicit executor for Domain 0
        self.sim_executor = SingleThreadedExecutor(context=self.sim_context)
        self.sim_executor.add_node(self.sim_node)
        
        # Publisher to pass those velocities directly to the digital twin's wheels
        self.sim_vel_pub = self.sim_node.create_publisher(Twist, '/cmd_vel', 10)
        
        print("------------------------------------------------------------")
        print("Jazzy Velocity Synchronization Bridge Active!")
        print("Sniffing Physical Motor Signals (Domain 30) -> Driving Digital Wheels (Domain 0)")
        print("------------------------------------------------------------")

    def velocity_callback(self, msg):
        # This function catches the exact speed and turn-rate of the physical robot
        # and instantly publishes it to the digital simulation robot's motors.
        self.sim_vel_pub.publish(msg)

    def spin(self):
        try:
            while rclpy.ok(context=self.phys_context) and rclpy.ok(context=self.sim_context):
                # Cycle data through both network domains concurrently
                self.phys_executor.spin_once(timeout_sec=0.01)
                self.sim_executor.spin_once(timeout_sec=0.01)
        except KeyboardInterrupt:
            print("\nShutting down Velocity Bridge cleanly...")
        finally:
            self.phys_executor.shutdown()
            self.sim_executor.shutdown()
            self.phys_node.destroy_node()
            self.sim_node.destroy_node()
            rclpy.shutdown(context=self.phys_context)
            rclpy.shutdown(context=self.sim_context)

def main():
    bridge = VelocitySyncBridge()
    bridge.spin()

if __name__ == '__main__':
    main()