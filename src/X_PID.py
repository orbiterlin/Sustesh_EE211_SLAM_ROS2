#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np

class XPIDController(Node):
    def __init__(self):
        super().__init__('x_pid_controller')
        
        # 参数
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('max_angular_vel', 0.5)
        self.declare_parameter('target_x', 0.0342)
        self.declare_parameter('tolerance', 0.005)
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.target_x = self.get_parameter('target_x').value
        self.tolerance = self.get_parameter('tolerance').value
        
        # PID变量
        self.prev_error = 0.0
        self.integral = 0.0
        
        # 订阅和发布
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco/pose',
            self.pose_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.current_x = None
        self.last_pose_time = None
        
        self.get_logger().info('X_PID控制器已启动')
        
    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.last_pose_time = self.get_clock().now()
        
    def control_loop(self):
        if self.current_x is None:
            return
            
        # 计算误差
        error = self.target_x - self.current_x
        
        # PID计算
        self.integral += error * 0.1
        derivative = (error - self.prev_error) / 0.1
        
        angular_vel = self.kp * error + self.ki * self.integral + self.kd * derivative
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))
        
        # 发布速度
        cmd = Twist()
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
        
        self.prev_error = error
        
        self.get_logger().info(f'X: {self.current_x:.4f}, 目标: {self.target_x}, 角速度: {angular_vel:.2f}')
        
        # 如果接近目标，停止
        if abs(error) < self.tolerance:
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info('X轴校准完成')

def main(args=None):
    rclpy.init(args=args)
    node = XPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
