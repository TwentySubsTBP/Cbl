import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json

# Just subscribes to the obstacle info just to make sure it's publishing right
class ObstacleInfoSubscriber(Node):
    def __init__(self):
        super().__init__('obstacle_info_subscriber')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            String,
            'obstacle_info',
            self.obstacle_info_callback,
            qos_profile
        )
    
    def obstacle_info_callback(self, msg):
        print("Received obstacle information: {}".format(msg.data))
def main(args=None):
    rclpy.init(args=args)

    obstacle_info_subscriber = ObstacleInfoSubscriber()
    
    rclpy.spin(obstacle_info_subscriber)
    
    obstacle_info_subscriber.destroy_node()
    
    rclpy.shutdown()