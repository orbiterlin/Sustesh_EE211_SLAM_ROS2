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
    
    # 配置文件路径
    cartographer_config_dir = PathJoinSubstitution([pkg_share, 'config'])
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 声明参数
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    
    # 启动激光雷达驱动 (RPLidar)
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }]
    )
    
    # 启动Cartographer SLAM
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer_params.lua'
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu')
        ]
    )
    
    # 启动Cartographer占用栅格节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.05,
        }]
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
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)
    
    return ld
