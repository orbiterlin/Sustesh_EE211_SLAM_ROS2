#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('EI_Final_Project').find('EI_Final_Project')
    
    # 参数声明
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 声明参数
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    
    ld.add_action(DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'))
    
    # 启动激光雷达驱动 (RPLidar) - 使用更灵活的参数
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }]
    )
    
    # 启动SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_params.yaml'])
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu')
        ]
    )
    
    # 启动RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'slam_2d.rviz'])]
    )
    
    # 添加节点到启动描述
    ld.add_action(lidar_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    
    return ld
