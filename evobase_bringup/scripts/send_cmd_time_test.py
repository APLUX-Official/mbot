#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一直发送cmdvel，线速度x=0.5，测试小车续航
持续发送运动命令，用于测试机器人的续航能力
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelTestNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_test_node')
        
        # 创建发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 创建定时器，10Hz频率发送命令
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)
        
        # 记录开始时间
        self.start_time = time.time()
        
        # 创建Twist消息
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0.5  # 线速度 0.5 m/s
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0  # 角速度为0，直线运动
        
        self.get_logger().info('开始续航测试，线速度: 0.5 m/s')
        self.get_logger().info('按 Ctrl+C 停止测试')
        
        # 计数器
        self.msg_count = 0
        
    def publish_cmd_vel(self):
        """发布cmd_vel消息"""
        self.cmd_vel_pub.publish(self.twist_msg)
        self.msg_count += 1
        
        # 每100次发布打印一次信息（每10秒）
        if self.msg_count % 100 == 0:
            elapsed_time = time.time() - self.start_time
            minutes = int(elapsed_time // 60)
            seconds = int(elapsed_time % 60)
            self.get_logger().info(f'续航测试进行中... 已运行: {minutes}分{seconds}秒, 发送消息数: {self.msg_count}')
    
    def destroy_node(self):
        """节点销毁时停止机器人"""
        # 发送停止命令
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
        # 计算总运行时间
        elapsed_time = time.time() - self.start_time
        minutes = int(elapsed_time // 60)
        seconds = int(elapsed_time % 60)
        
        self.get_logger().info(f'续航测试结束！')
        self.get_logger().info(f'总运行时间: {minutes}分{seconds}秒')
        self.get_logger().info(f'总发送消息数: {self.msg_count}')
        self.get_logger().info('机器人已停止')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = CmdVelTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到停止信号...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()