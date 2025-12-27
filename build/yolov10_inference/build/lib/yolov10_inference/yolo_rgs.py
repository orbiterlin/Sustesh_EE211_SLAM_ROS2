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
        
        # 初始化 YOLOv10 模型
        self.model = YOLO('/home/tony/ros2_ws/src/yolov10_inference/yolov10_inference/models/best1.pt')
        #self.bounding_box_annotator = sv.BoundingBoxAnnotator()
        self.bounding_box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        self.bridge = CvBridge()
	
	# 发布检测结果
        self.result_publisher = self.create_publisher(String, 'yolo_detection_results', 10)
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # 替换为实际图像话题名称
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # 推理
        results = self.model(frame)[0]
        detections = sv.Detections.from_ultralytics(results)
	
	# 如果有检测结果
        if len(detections.class_id) > 0:
            detected_objects = []
            for class_id, confidence in zip(detections.class_id, detections.confidence):
                label = self.model.names[class_id]
                detected_objects.append(f'{label} ({confidence:.2f})')
        
        	# 将检测结果拼接成字符串
            detection_message = ", ".join(detected_objects)
            self.result_publisher.publish(String(data=detection_message))
            self.get_logger().info(f'Detected: {detection_message}')
        else:
            detection_message = "none (1.00)"
            self.result_publisher.publish(String(data=detection_message))
            self.get_logger().info('No objects detected.')
        
        # 标注图像
        annotated_image = self.bounding_box_annotator.annotate(scene=frame, detections=detections)
        annotated_image = self.label_annotator.annotate(scene=annotated_image, detections=detections)

        # 显示标注图像
        #cv2.imshow('YOLOv10 Detection', annotated_image)
        #cv2.waitKey(1)

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

