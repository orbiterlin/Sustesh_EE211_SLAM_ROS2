import warnings
warnings.filterwarnings('ignore', category=UserWarning, message='.*Axes3D.*')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import supervision as sv
from ultralytics import YOLO
from std_msgs.msg import String

class YOLOv10Node(Node):
    def __init__(self):
        super().__init__('yolov10_node')
        
        # 声明参数
        self.declare_parameter('confidence_threshold', 0.5)  # 置信度阈值
        self.declare_parameter('target_classes', ['red', 'green', 'stop'])  # 只关注这些类别
        
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.target_classes = self.get_parameter('target_classes').value
        
        # 初始化 YOLOv10 模型
        self.model = YOLO('/home/tony/ros2_ws/src/yolov10_inference/yolov10_inference/models/best1.pt')
        self.bounding_box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.bridge = CvBridge()
	
	# 发布检测结果
        self.result_publisher = self.create_publisher(String, 'yolo_detection_results', 10)
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info(f'YOLO节点已启动，置信度阈值: {self.conf_threshold}')
        self.get_logger().info(f'目标类别: {self.target_classes}')

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 推理，设置置信度阈值
        results = self.model(frame, conf=self.conf_threshold)[0]
        detections = sv.Detections.from_ultralytics(results)
	
	# 过滤检测结果：只保留目标类别和高置信度的
        detected_objects = []
        if len(detections.class_id) > 0:
            for class_id, confidence in zip(detections.class_id, detections.confidence):
                label = self.model.names[class_id].lower()
                
                # 只关注目标类别
                if any(target in label for target in self.target_classes):
                    # 再次检查置信度（双重保险）
                    if confidence >= self.conf_threshold:
                        detected_objects.append(f'{label} ({confidence:.2f})')
        
        # 发布检测结果
        if detected_objects:
            detection_message = ", ".join(detected_objects)
            self.result_publisher.publish(String(data=detection_message))
            self.get_logger().info(f'✅ 检测到: {detection_message}')
        else:
            detection_message = "none (1.00)"
            self.result_publisher.publish(String(data=detection_message))
        
        # 标注图像（可选，用于调试）
        # annotated_image = self.bounding_box_annotator.annotate(scene=frame, detections=detections)
        # annotated_image = self.label_annotator.annotate(scene=annotated_image, detections=detections)
        # cv2.imshow('YOLOv10 Detection', annotated_image)
        # cv2.waitKey(1)

    def destroy(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv10Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

