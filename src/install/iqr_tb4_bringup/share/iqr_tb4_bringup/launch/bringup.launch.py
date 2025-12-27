# -*- coding: utf-8 -*-
"""
文件：bringup.launch.py
维护者(Maintainer): furrypiglet <qian.huang@iqr-robot.com>
变更要点：
  - 将原有的“机械臂相关驱动”替换为：**直接调用 /ax12a_arm_controller/launch/ax12a_arm.launch.py**
  - 保留原有 URDF、TF、IMU、RPLIDAR、RealSense、云台、RViz 的启动逻辑

使用示例：
  ros2 launch iqr_tb4_bringup bringup.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    功能（函数级中文注释）：
    1. 加载整机 URDF 并由 robot_state_publisher 发布 TF（保持与实际硬件一致的关节树）。
    2. 发布左右轮跌落传感器的静态 TF（与底盘几何位置固定关系）。
    3. 启动 RViz 进行可视化（可通过 bringup_use_rviz 打开/关闭）。
    4. 启动 IMU、RPLIDAR、RealSense、云台等外设的各自 bringup。
    5. **机械臂部分：不再在此传参，改为直接包含 `/ax12a_arm_controller/launch/ax12a_arm.launch.py`。
       若需要配置串口/波特率/ID，请直接在该子 launch 内进行，避免多处维护。**
    设计理念：
    - 上层 bringup 只承担“装配与编排”；每个子系统的参数与实现放到自己包里，降低耦合。
    """
    # ----------------------------- Share 路径区 -----------------------------
    pkg_share = FindPackageShare(package='iqr_tb4_description').find('iqr_tb4_description')
    iqr_tb4_share = FindPackageShare(package='iqr_tb4_bringup').find('iqr_tb4_bringup')
    witmotion_share = FindPackageShare(package='witmotion_ros_driver').find('witmotion_ros_driver')
    rplidar_share = FindPackageShare(package='rplidar_ros').find('rplidar_ros')
    pan_tilt_share = FindPackageShare(package='pan_tilt_bringup').find('pan_tilt_bringup')
    realsense_share = FindPackageShare(package='realsense2_camera').find('realsense2_camera')
    ax12a_ctrl_share = FindPackageShare(package='ax12a_arm_controller').find('ax12a_arm_controller')

    # ----------------------------- 路径/文件区 -----------------------------
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_basic_settings.rviz')
    imu_bringup_path = os.path.join(witmotion_share, 'launch/modbus_rtu_driver.launch.py')
    imu2_bringup_path = os.path.join(witmotion_share, 'launch/serial_bridge.launch.py')
    rplidar_bringup_path = os.path.join(rplidar_share, 'launch/rplidar_a2m12_launch.py')
    realsense_bringup_path = os.path.join(realsense_share, 'launch/rs_launch.py')
    pan_tilt_bringup_path = os.path.join(pan_tilt_share, 'launch/panTilt_bringup.launch.py')
    ax12a_bringup_path = os.path.join(ax12a_ctrl_share, 'launch/ax12a_arm.launch.py')

    default_urdf_model_path = os.path.join(pkg_share, 'urdf/tb_full.urdf.xacro')

    # ----------------------------- 启动参数区 -----------------------------
    gui = LaunchConfiguration('gui')
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    bringup_use_rviz = LaunchConfiguration('bringup_use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ----------------------------- 参数声明区 -----------------------------
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui'
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Whether to start the robot state publisher'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='bringup_use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # ----------------------------- Node / Include 定义区 -----------------------------
    # 1) robot_state_publisher：根据 URDF 发布 TF
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_model])
        }],
        arguments=[default_urdf_model_path],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # 2) 左/右跌落开关 TF（与底盘几何固定）
    left_wheel_drop_stf = Node(
        name='left_wheel_drop_stf',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--x', '0.0', '--y', '0.1165', '--z', '0.0402',
                   '--roll', '-1.5707', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'wheel_drop_left'],
        remappings=[('/tf_static', 'tf_static')],
    )
    right_wheel_drop_stf = Node(
        name='right_wheel_drop_stf',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['--x', '0.0', '--y', '-0.1165', '--z', '0.0402',
                   '--roll', '-1.5707', '--pitch', '0.0', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'wheel_drop_right'],
        remappings=[('/tf_static', 'tf_static')],
    )

    # 3) RViz 可视化
    start_rviz_cmd = Node(
        condition=IfCondition(bringup_use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 4) 传感器/外设 bringup
    imu2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([imu2_bringup_path])
    )
    imu_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([imu_bringup_path])
    )
    rplidar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rplidar_bringup_path]),
        launch_arguments={
            'serial_port': '/dev/rplidar',
            'serial_baudrate': '256000',
            'frame_id': 'laser_link',
            'inverted': 'false'
        }.items()
    )
    realsense_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_bringup_path]),
        launch_arguments={'pointcloud.enable': 'true', 'publish_tf': 'true'}.items()
    )
    pan_tilt_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pan_tilt_bringup_path])
    )

    # 5) 机械臂（AX-12A）：**直接包含 ax12a_arm_controller 的 launch，不在此重复传参**
    ax12a_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ax12a_bringup_path])
    )

    # ----------------------------- LaunchDescription 组装区 -----------------------------
    ld = LaunchDescription()

    # 声明参数
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # 挂载各子系统（顺序无硬性要求）
    ld.add_action(ax12a_action)                 # ✅ 机械臂（AX-12A）
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(imu2_bringup)
    ld.add_action(imu_bringup)
    ld.add_action(rplidar_bringup)
    ld.add_action(pan_tilt_bringup)
    ld.add_action(realsense_bringup)
    ld.add_action(left_wheel_drop_stf)
    ld.add_action(right_wheel_drop_stf)
    ld.add_action(start_rviz_cmd)

    return ld

