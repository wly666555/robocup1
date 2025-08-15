#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import MotorCmd, MotorStates
import time

class TestServoControl(Node):
    def __init__(self):
        super().__init__('test_servo_control')
        
        # 创建发布者 - 发送电机命令
        self.motor_cmd_pub = self.create_publisher(
            MotorCmd, 
            'servo/motor_cmd', 
            10
        )
        
        # 创建订阅者 - 接收电机状态
        self.motor_states_sub = self.create_subscription(
            MotorStates,
            'servo/motor_states',
            self.motor_states_callback,
            10
        )
        
        # 创建定时器 - 定期发送命令
        self.timer = self.create_timer(0.1, self.send_motor_command)
        
        self.get_logger().info('Test servo control node started')
        
    def motor_states_callback(self, msg):
        self.get_logger().info(f'Received motor states: q={msg.q}, mode={msg.mode}, dq={msg.dq}')
        
    def send_motor_command(self):
        # 创建电机命令消息
        cmd = MotorCmd()
        cmd.mode = 1  # 启用模式
        cmd.q = 45.0  # 目标位置45度
        cmd.kp = 500.0  # 位置增益
        cmd.kd = 300.0  # 速度增益
        cmd.tau = 0.0   # 力矩
        cmd.reserve = [0] * 8  # 保留字段
        
        # 发布命令
        self.motor_cmd_pub.publish(cmd)
        self.get_logger().info(f'Sent motor command: q={cmd.q}, mode={cmd.mode}')

def main(args=None):
    rclpy.init(args=args)
    
    test_node = TestServoControl()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 