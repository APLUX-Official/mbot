#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_converter')
        
        # 订阅 TwistStamped 话题
        self.sub = self.create_subscription(
            TwistStamped, 
            '/cmd_vel_smoothed',  # 从 smoothed 话题订阅
            self.callback, 
            10)
        
        # 发布 Twist 话题
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Twist converter started')
    
    def callback(self, msg: TwistStamped):
        # 提取 Twist 部分并转发
        self.pub.publish(msg.twist)

def main(args=None):
    rclpy.init(args=args)
    node = TwistConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
