#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('EI_Final_Project').find('EI_Final_Project')
    
    # 参数声明
    map_name = LaunchConfiguration('map_name', default='my_map')
    maps_dir = PathJoinSubstitution([pkg_share, 'maps'])
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 声明参数
    ld.add_action(DeclareLaunchArgument(
        'map_name',
        default_value='my_map',
        description='Name of the map to save (without extension)'))
    
    # 创建地图目录
    mkdir_cmd = ExecuteProcess(
        cmd=['mkdir', '-p', maps_dir],
        output='screen'
    )
    
    # 保存地图服务
    map_saver_server = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        output='screen',
        parameters=[{
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'map_subscribe_transient_local': True,
        }]
    )
    
    # 保存地图客户端
    map_saver_client = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/map_saver/save_map',
            'nav2_msgs/srv/SaveMap',
            f'{{map_url: "{maps_dir}/{map_name}", map_mode: "trinary", image_format: "pgm", free_thresh: 0.25, occupied_thresh: 0.65}}'
        ],
        output='screen',
        shell=True
    )
    
    # 延迟执行保存命令
    delayed_save = TimerAction(
        period=2.0,
        actions=[map_saver_client]
    )
    
    # 添加所有动作
    ld.add_action(mkdir_cmd)
    ld.add_action(map_saver_server)
    ld.add_action(delayed_save)
    
    return ld
