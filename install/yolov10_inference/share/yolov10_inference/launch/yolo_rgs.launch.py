from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov10_inference',  
            executable='yolo_rgs',       
            name='yolo_rgs_node',        # 设置节点名称
            output='screen',             # 输出日志到屏幕
            parameters=[]                # 如果需要，可以添加参数
        )
    ])

