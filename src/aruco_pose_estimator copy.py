#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimator')
        
        # 1. 初始化参数
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # 匹配生成的ArUco字典
        self.aruco_params = aruco.DetectorParameters()
        self.bridge = CvBridge()
        self.aruco_size = 0.03  # ArUco码物理尺寸（单位：米，需与实际打印一致）
        
        # 2. 相机参数（初始值，会被CameraInfo覆盖）
        self.camera_matrix = np.array([[615.0, 0.0, 320.0],
                                       [0.0, 615.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # 畸变系数
        
        # 3. 订阅相机图像（TurtleBot4 Realsense话题）
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Realsense彩色图像话题
            self.image_callback,
            10)
        
        # 4. 订阅相机内参（自动获取标定参数）
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10)
        
        # 5. 发布ArUco位姿
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        
        self.get_logger().info("ArUco姿态估计节点已启动")

    def camera_info_callback(self, msg):
        # 从CameraInfo自动更新相机内参
        self.camera_matrix = np.reshape(msg.k, (3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        try:
            # 将ROS图像转为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return
        
        # 1. 检测ArUco码（仅传图像、字典、检测参数，移除相机参数）
        corners, ids, rejected = aruco.detectMarkers(
            cv_image, 
            self.aruco_dict, 
            parameters=self.aruco_params  # 仅保留检测参数
        )
        
        # 2. 若检测到ArUco，进行姿态估计
        if ids is not None and len(ids) > 0:
            # 绘制检测到的ArUco码（可视化）
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            # 对每个检测到的ArUco解算位姿
            for i in range(len(ids)):
                # PnP解算旋转向量(rvec)和平移向量(tvec)（此处传入相机参数）
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i], 
                    self.aruco_size,
                    cameraMatrix=self.camera_matrix,  # 位姿估计时传相机参数
                    distCoeffs=self.dist_coeffs
                )
                
                # 绘制坐标轴（可视化位姿，长度=aruco_size）
                cv2.drawFrameAxes(
                    cv_image, 
                    self.camera_matrix, 
                    self.dist_coeffs, 
                    rvec, 
                    tvec, 
                    self.aruco_size
                )
                
                # 3. 将位姿转为ROS PoseStamped消息发布
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_color_optical_frame"  # 相机坐标系
                
                # 平移向量（x/y/z，单位：米）
                pose_msg.pose.position.x = float(tvec[0][0][0])
                pose_msg.pose.position.y = float(tvec[0][0][1])
                pose_msg.pose.position.z = float(tvec[0][0][2])
                
                # 旋转向量转四元数（OpenCV→ROS）
                rvec_mat = cv2.Rodrigues(rvec)[0]  # 旋转向量转旋转矩阵
                quat = self.rotation_matrix_to_quaternion(rvec_mat)
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                
                # 发布位姿
                self.pose_pub.publish(pose_msg)
                self.get_logger().info(f"检测到ArUco ID: {ids[i][0]}, 平移: {tvec[0][0]}")
        
        # 显示结果图像（需确保有GUI环境，如SSH转发X11或本地运行）
        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    def rotation_matrix_to_quaternion(self, R):
        """旋转矩阵转四元数（x,y,z,w）"""
        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
        qx = (R[2,1] - R[1,2]) / (4 * qw)
        qy = (R[0,2] - R[2,0]) / (4 * qw)
        qz = (R[1,0] - R[0,1]) / (4 * qw)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
