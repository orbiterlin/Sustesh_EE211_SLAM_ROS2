#!/usr/bin/env python3
"""
视觉检测与导航协调节点
订阅YOLO检测结果，根据检测模式发布导航控制信号（PAUSE/RESUME）
不直接控制cmd_vel，避免与导航系统冲突
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionNavCoordinator(Node):
    def __init__(self):
        super().__init__('vision_nav_coordinator')
        
        # 声明参数：检测模式
        self.declare_parameter('detection_mode', 'traffic_light')  # 'traffic_light' 或 'stop_sign'
        self.detection_mode = self.get_parameter('detection_mode').value
        
        # 订阅 YOLO 检测结果
        self.yolo_sub = self.create_subscription(
            String,
            '/yolo_detection_results',
            self.yolo_callback,
            10
        )
        
        # 发布导航控制信号
        self.nav_control_pub = self.create_publisher(
            String,
            '/vision/nav_control',
            1
        )
        
        # 状态管理
        self.current_state = 'RESUME'  # RESUME 或 PAUSE
        self.last_published_state = None
        
        # 防抖计数
        self.stop_count = 0
        self.go_count = 0
        self.confirm_frames = 5  # 连续5帧确认
        
        # 无stop检测计数（用于stop牌子模式）
        self.no_stop_count = 0
        self.no_stop_threshold = 10  # 连续10帧无stop才恢复
        
        self.get_logger().info(f'视觉导航协调节点已启动，模式: {self.detection_mode}')
    
    def set_detection_mode(self, mode):
        """切换检测模式"""
        if mode in ['traffic_light', 'stop_sign']:
            self.detection_mode = mode
            self.stop_count = 0
            self.go_count = 0
            self.no_stop_count = 0
            self.current_state = 'RESUME'
            self.get_logger().info(f'切换检测模式为: {mode}')
        else:
            self.get_logger().warn(f'无效的检测模式: {mode}')
    
    def yolo_callback(self, msg: String):
        """处理YOLO检测结果"""
        text = msg.data.lower()
        new_state = None
        
        if self.detection_mode == 'traffic_light':
            # 交通灯模式：检测红灯/绿灯
            if 'red' in text:
                self.stop_count += 2
                self.go_count = 0
                self.get_logger().debug(f'检测到红色，stop_count={self.stop_count}, go_count={self.go_count}')
            elif 'green' in text:
                self.go_count += 2
                self.stop_count = 0
                self.get_logger().debug(f'检测到绿色，stop_count={self.stop_count}, go_count={self.go_count}')
            else:
                # 不确定情况（none或其他），保持当前状态
                # 如果当前是暂停状态，保持暂停；如果当前是运行状态，保持运行
                # 不改变计数，避免误判
                self.get_logger().debug(f'未检测到红绿灯，保持当前状态: {self.current_state}')
            
            # 状态切换
            if self.current_state == 'RESUME' and self.stop_count >= self.confirm_frames:
                new_state = 'PAUSE'
                self.get_logger().info('🛑 检测到红灯，暂停导航')
            elif self.current_state == 'PAUSE' and self.go_count >= self.confirm_frames:
                new_state = 'RESUME'
                self.get_logger().info('🟢 检测到绿灯，恢复导航')
        
        elif self.detection_mode == 'stop_sign':
            # Stop牌子模式：检测stop牌子
            if 'stop' in text:
                self.stop_count += 2
                self.go_count = 0
                self.no_stop_count = 0
                self.get_logger().debug(f'检测到stop牌子，stop_count={self.stop_count}, no_stop_count={self.no_stop_count}')
            else:
                # 没有检测到stop
                self.stop_count = max(0, self.stop_count - 1)
                self.no_stop_count += 1
                self.get_logger().debug(f'未检测到stop牌子，stop_count={self.stop_count}, no_stop_count={self.no_stop_count}')
            
            # 状态切换
            if self.current_state == 'RESUME' and self.stop_count >= self.confirm_frames:
                new_state = 'PAUSE'
                self.get_logger().info('🛑 检测到stop牌子，暂停导航')
            elif self.current_state == 'PAUSE' and self.no_stop_count >= self.no_stop_threshold:
                new_state = 'RESUME'
                self.get_logger().info('✅ 未检测到stop牌子，恢复导航')
        
        # 发布状态变化
        if new_state and new_state != self.current_state:
            self.current_state = new_state
            self.publish_nav_control(new_state)
    
    def publish_nav_control(self, state):
        """发布导航控制信号"""
        if state != self.last_published_state:
            msg = String()
            msg.data = state
            self.nav_control_pub.publish(msg)
            self.last_published_state = state
            self.get_logger().info(f'发布导航控制: {state}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNavCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

