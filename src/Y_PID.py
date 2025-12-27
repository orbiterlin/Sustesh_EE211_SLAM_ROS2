#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np

class YPIDController(Node):
    def __init__(self):
        super().__init__('y_pid_controller')
        
        # 参数
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('max_linear_vel', 0.3)
        self.declare_parameter('target_y', 0.0048)
        self.declare_parameter('tolerance', 0.002)
        
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.target_y = self.get_parameter('target_y').value
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
        
        self.current_y = None
        self.last_pose_time = None
        
        self.get_logger().info('Y_PID控制器已启动')
        
    def pose_callback(self, msg):
        self.current_y = msg.pose.position.y
        self.last_pose_time = self.get_clock().now()
        
    def control_loop(self):
        if self.current_y is None:
            return
            
        # 计算误差
        error = self.target_y - self.current_y
        
        # PID计算
        self.integral += error * 0.1
        derivative = (error - self.prev_error) / 0.1
        
        linear_vel = self.kp * error + self.ki * self.integral + self.kd * derivative
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
        
        # 发布速度
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        self.prev_error = error
        
        self.get_logger().info(f'Y: {self.current_y:.4f}, 目标: {self.target_y}, 线速度: {linear_vel:.2f}')
        
        # 如果接近目标，停止
        if abs(error) < self.tolerance:
            cmd.linear.x = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info('Y轴校准完成')

def main(args=None):
    rclpy.init(args=args)
    node = YPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
