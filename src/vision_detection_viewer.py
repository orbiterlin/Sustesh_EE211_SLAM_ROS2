#!/usr/bin/env python3
"""
视觉检测结果可视化节点
订阅YOLO检测结果，在终端以彩色输出显示，方便观察检测效果
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VisionDetectionViewer(Node):
    def __init__(self):
        super().__init__('vision_detection_viewer')
        
        # 订阅 YOLO 检测结果
        self.yolo_sub = self.create_subscription(
            String,
            '/yolo_detection_results',
            self.yolo_callback,
            10
        )
        
        # 可选：订阅图像用于可视化（如果有GUI环境）
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.last_detection = "none"
        self.detection_count = 0
        
        # 检测统计
        self.red_count = 0
        self.green_count = 0
        self.stop_count = 0
        self.none_count = 0
        
        self.get_logger().info('视觉检测可视化节点已启动')
        self.get_logger().info('=' * 60)
        self.get_logger().info('等待检测结果...')
        self.get_logger().info('=' * 60)
    
    def yolo_callback(self, msg: String):
        """处理YOLO检测结果"""
        text = msg.data.lower()
        
        # 统计检测结果
        has_red = 'red' in text
        has_green = 'green' in text
        has_stop = 'stop' in text
        is_none = 'none' in text or text.strip() == ''
        
        # 更新统计
        if has_red:
            self.red_count += 1
        elif has_green:
            self.green_count += 1
        elif has_stop:
            self.stop_count += 1
        elif is_none:
            self.none_count += 1
        
        self.detection_count += 1
        
        # 格式化输出
        if has_red:
            # 红色高亮显示红灯
            status = f"\033[91m🔴 检测到红灯: {msg.data}\033[0m"
            self.get_logger().info(status)
        elif has_green:
            # 绿色高亮显示绿灯
            status = f"\033[92m🟢 检测到绿灯: {msg.data}\033[0m"
            self.get_logger().info(status)
        elif has_stop:
            # 黄色高亮显示stop牌子
            status = f"\033[93m🛑 检测到STOP牌子: {msg.data}\033[0m"
            self.get_logger().info(status)
        else:
            # 普通显示无检测
            if self.detection_count % 10 == 0:  # 每10帧显示一次，避免刷屏
                status = f"⚪ 未检测到目标: {msg.data}"
                self.get_logger().info(status)
        
        self.last_detection = text
        
        # 每50帧显示一次统计信息
        if self.detection_count % 50 == 0:
            self.print_statistics()
    
    def print_statistics(self):
        """打印检测统计信息"""
        total = self.detection_count
        self.get_logger().info('=' * 60)
        self.get_logger().info('📊 检测统计 (最近50帧):')
        self.get_logger().info(f'   🔴 红灯: {self.red_count} 次')
        self.get_logger().info(f'   🟢 绿灯: {self.green_count} 次')
        self.get_logger().info(f'   🛑 STOP: {self.stop_count} 次')
        self.get_logger().info(f'   ⚪ 无检测: {self.none_count} 次')
        self.get_logger().info('=' * 60)
        
        # 重置统计（滑动窗口）
        self.red_count = 0
        self.green_count = 0
        self.stop_count = 0
        self.none_count = 0
    
    def image_callback(self, msg: Image):
        """处理图像（可选，如果有GUI环境可以显示图像）"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 在图像上添加检测信息
            detection_text = self.last_detection
            if 'red' in detection_text:
                color = (0, 0, 255)  # 红色
                text = "RED DETECTED"
            elif 'green' in detection_text:
                color = (0, 255, 0)  # 绿色
                text = "GREEN DETECTED"
            elif 'stop' in detection_text:
                color = (0, 165, 255)  # 橙色
                text = "STOP DETECTED"
            else:
                color = (128, 128, 128)  # 灰色
                text = "NO DETECTION"
            
            # 在图像上绘制文本
            cv2.putText(cv_image, text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.putText(cv_image, f"Frame: {self.detection_count}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 显示图像（需要GUI环境）
            cv2.imshow('Vision Detection Viewer', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            # 如果没有GUI环境，忽略图像显示错误
            pass


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectionViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('关闭可视化节点...')
    finally:
        try:
            cv2.destroyAllWindows()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


