#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MotorCmd, MotorStates
import time

class ServoTestNode(Node):
    def __init__(self):
        super().__init__('servo_test_node')
        
        # 创建发布者和订阅者
        self.cmd_publisher = self.create_publisher(MotorCmd, 'servo/motor_cmd', 10)
        self.state_subscription = self.create_subscription(
            MotorStates, 'servo/motor_states', self.state_callback, 10)
        
        # 创建定时器，每秒发送一次命令
        self.timer = self.create_timer(1.0, self.send_command)
        
        self.get_logger().info('Servo test node started')
        
    def send_command(self):
        """发送电机控制命令"""
        cmd = MotorCmd()
        cmd.mode = 1  # 启用电机
        cmd.q = 45.0  # 目标位置45度
        cmd.kp = 500.0  # 位置增益
        cmd.kd = 300.0  # 速度增益
        cmd.tau = 0.0  # 力矩
        cmd.reserve = [0, 0, 0, 0, 0, 0, 0, 0]  # 保留字段
        
        self.cmd_publisher.publish(cmd)
        self.get_logger().info(f'Sent command: mode={cmd.mode}, q={cmd.q}, kp={cmd.kp}, kd={cmd.kd}')
        
    def state_callback(self, msg):
        """接收电机状态回调"""
        self.get_logger().info(f'Received state: mode={msg.mode}, q={msg.q}, dq={msg.dq}, tau_est={msg.tau_est}')

def main():
    rclpy.init()
    node = ServoTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 