#!/usr/bin/env python3

import rclpy
import random
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def main():
    rclpy.init()

    # Initialize the Simple Commander Navigator
    navigator = BasicNavigator()

    # 1. Wait for Nav2 to completely boot up and be ready
    print("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    print("Nav2 is active and ready to roll!")

    # 2. Define your goal pose (X, Y coordinates on your map)
    # Looking at your map screenshot, a safe clear spot right in front 
    # of the starting point might be around X=1.0, Y=0.0. Adjust as needed!
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # --- CHANGE THESE VALUES TO TEST DIFFERENT DESTINATIONS ---
    goal_pose.pose.position.x =round(random.uniform(-1, 1),2 )  # 1 meter forward
    goal_pose.pose.position.y = round(random.uniform(-1, 3),2 )  # 0 meters sideways
    # -----------------------------------------------------------
    
    goal_pose.pose.orientation.w = 1.0 # Facing straight ahead

    path = navigator.getPath(initial_pose=None, goal_pose=goal_pose)

    if path is None:
        print("\n[CRITICAL ERROR]: Destination is UNREACHABLE or OUT OF BOUNDS!")
        print("Aborting mission safely. Robot will remain stationary.")
        rclpy.shutdown()
        sys.exit(1) # Exit script safely with an error code

    # 3. Send the robot to the goal
    print(f"Sending robot to goal coordinates: X={goal_pose.pose.position.x}, Y={goal_pose.pose.position.y}")
    navigator.goToPose(goal_pose)

    # 4. Monitor the progress of the robot
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            # Print remaining distance every few seconds
            print(f"Distance remaining: {feedback.distance_remaining:.2f} meters.")

    # 5. Check the final outcome
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Success! The robot reached its destination!")
    elif result == TaskResult.CANCELED:
        print("The navigation goal was canceled.")
    elif result == TaskResult.FAILED:
        print("The navigation goal failed!")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()