import rclpy
from rclpy.node import Node
import cv2
import supervision as sv
from ultralytics import YOLO

class YOLOv10Node(Node):
    def __init__(self):
        super().__init__('yolov10_node')
        
        # 初始化模型
        self.model = YOLO('/home/tony/ros2_ws/src/yolov10_inference/yolov10_inference/models/best1.pt')
        self.bounding_box_annotator = sv.BoundingBoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

        # 打开摄像头
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        if not self.cap.isOpened():
            self.get_logger().error("Can't open the webcam")
            return

        # 启动定时器，定期处理视频帧
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to grab frame")
            return

        # 推理
        results = self.model(frame)[0]
        detections = sv.Detections.from_ultralytics(results)

        # 标注图像
        annotated_image = self.bounding_box_annotator.annotate(scene=frame, detections=detections)
        annotated_image = self.label_annotator.annotate(scene=annotated_image, detections=detections)

        # 显示标注图像
        cv2.imshow('Webcam', annotated_image)
        cv2.waitKey(1)

    def destroy(self):
        self.cap.release()
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

