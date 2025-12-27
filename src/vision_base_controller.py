#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class VisionBaseController(Node):
    def __init__(self):
        super().__init__('vision_base_controller')

        # 订阅 YOLO 结果
        self.sub = self.create_subscription(
            String,
            '/yolo_detection_results',
            self.yolo_callback,
            10
        )

        # 发布底盘速度（⚠️ 控制话题，队列建议小）
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        # ---------- 状态 ----------
        self.state = 'GO'   # GO / STOP

        # ---------- 防抖计数 ----------
        self.stop_count = 0
        self.go_count = 0
        self.confirm_frames = 5  # 连续 5 帧确认

        # ---------- 速度参数 ----------
        self.forward_speed = 0.1

        self.get_logger().info('Vision base controller with state machine started.')

    def yolo_callback(self, msg: String):
        text = msg.data.lower()
        cmd = Twist()

        # ---------- STOP / GO 计数 ----------
        if 'stop' in text or 'red' in text:
            self.stop_count += 2
            self.go_count = 0
        elif 'green' in text:
            self.go_count += 2
            self.stop_count = 0
        else:
            # 不确定情况：不切状态
            self.stop_count = 0
            self.go_count += 1

        # ---------- 状态切换 ----------
        if self.state == 'GO' and self.stop_count >= self.confirm_frames:
            self.state = 'STOP'
            self.get_logger().info('🛑 Enter STOP state')

        elif self.state == 'STOP' and self.go_count >= self.confirm_frames:
            self.state = 'GO'
            self.get_logger().info('🟢 Enter GO state')

        # ---------- 状态输出 ----------
        if self.state == 'STOP':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VisionBaseController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前明确停车（非常重要）
        stop_cmd = Twist()
        node.pub.publish(stop_cmd)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
