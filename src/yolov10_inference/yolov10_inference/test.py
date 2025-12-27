import rclpy
from rclpy.node import Node
import cv2
import supervision as sv
from ultralytics import YOLO

class YOLOv10Node(Node):
    def __init__(self):
        super().__init__('yolov10_node')
        
        # 初始化模型
        self.model = YOLO('/home/tony/ros2_ws/src/yolov10_inference/yolov10_inference/models/best.pt')
        self.bounding_box_annotator = sv.BoxAnnotator()  # 使用新的BoxAnnotator
        self.label_annotator = sv.LabelAnnotator()

        # 载入图像
        self.image_path = '/home/tony/ros2_ws/src/yolov10_inference/yolov10_inference/image/output_images/opencv_frame_29.png'  # 设置要处理的图片路径
        self.image = cv2.imread(self.image_path)

        if self.image is None:
            self.get_logger().error(f"Can't open the image at {self.image_path}")
            return
        
        # 执行图像推理
        self.process_image()

    def process_image(self):
        # 推理
        results = self.model(self.image)[0]
        detections = sv.Detections.from_ultralytics(results)

        # 标注图像
        annotated_image = self.bounding_box_annotator.annotate(scene=self.image, detections=detections)
        annotated_image = self.label_annotator.annotate(scene=annotated_image, detections=detections)

        # 显示标注图像
        cv2.imshow('Annotated Image', annotated_image)
        cv2.waitKey(0)  # 按任意键关闭图像窗口

        # 自动退出程序
        self.get_logger().info("Processing complete. Shutting down...")
        rclpy.shutdown()

    def destroy(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv10Node()

    try:
        while rclpy.ok():  # 替换 rclpy.spin(node) 为主动检查 shutdown 状态
            rclpy.spin_once(node)  # 手动检查ROS节点的状态
    except KeyboardInterrupt:
        pass

    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


