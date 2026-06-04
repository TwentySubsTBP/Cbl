import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json
import math
class ObstacleInfo(Node):
    def __init__(self):
        super().__init__('obstacle_info')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile
        )
        
        self.publisher = self.create_publisher(
            String, 
            'obstacle_info', 
            qos_profile
        )
        
    
    def scan_callback(self, scan_data):
        num_ranges = len(scan_data.ranges)
        if num_ranges == 0:
            return
        
        q_range = num_ranges // 4  # split the number of scan points into four for each sector/dir

        #Based on what I found the list of readings is counter clockwise starting from directly in front
        # Hence why we must do this weird indexing, I left the additions there to make it easier to read
        front_range = list(scan_data.ranges[0:q_range//2]) + list(scan_data.ranges[-q_range//2:])
        left_range = list(scan_data.ranges[q_range//2: q_range//2 + q_range])
        back_range = list(scan_data.ranges[q_range//2 + q_range: q_range//2 + 2*q_range])
        right_range = list(scan_data.ranges[q_range//2 + 2*q_range: q_range//2 + 3*q_range])
        
        #Clean up the data by removing infinites and NaNs
        def clean_range(range):
            # keep only finite, positive readings (drops inf, NaN, and 0.0/invalid points)
            return [r for r in range if math.isfinite(r) and r > 0.0]
        
        front_range = clean_range(front_range)
        left_range = clean_range(left_range)
        back_range = clean_range(back_range)
        right_range = clean_range(right_range)

        # Calculate the minimum distance in each direction
        min_front = min(front_range) if front_range else float('inf')
        min_left = min(left_range) if left_range else float('inf')
        min_back = min(back_range) if back_range else float('inf')
        min_right = min(right_range) if right_range else float('inf')

        # Is there an obstacle within range in any dir
        range_threshold = 0.5  # Define a threshold distance for obstacle detection
        obstacle = min_front < range_threshold or min_left < range_threshold or min_back < range_threshold or min_right < range_threshold
        
        # Output info, in json cause its easiest to parse
        output = {
            'obstacle': obstacle,
            'min_front': round(min_front, 2) if min_front != float('inf') else 'clear',
            'min_left': round(min_left, 2) if min_left != float('inf') else 'clear',
            'min_back': round(min_back, 2) if min_back != float('inf') else 'clear',
            'min_right': round(min_right, 2) if min_right != float('inf') else 'clear'
        }

        string_output = String()
        string_output.data = json.dumps(output)

        # Publish the interpreted obstacle information
        
        self.publisher.publish(string_output)
        self.get_logger().info(f"Front: {output['min_front']}m | Obstacle Nearby: {output['obstacle']}")

def main(args=None):
    rclpy.init(args=args)

    obstacle_info_node = ObstacleInfo()

    rclpy.spin(obstacle_info_node)

    obstacle_info_node.destroy_node()

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()