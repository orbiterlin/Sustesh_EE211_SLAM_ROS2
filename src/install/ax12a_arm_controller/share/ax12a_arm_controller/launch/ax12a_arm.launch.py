from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ax12a_arm_controller',
            executable='ax12a_arm_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyDXL',
                'baudrate': 1000000,
                'protocol': 1.0,

                'ids_arm': [1,2,3,4],
                'id_gripper': 5,
                'joint_names_arm': ['joint1','joint2','joint3','joint4'],
                'joint_name_gripper': 'gripper',

                'zero_deg':  [150.0,150.0,150.0,150.0,  0.0],
                'direction': [  1.0, -1.0,  1.0, -1.0,  1.0],
                'min_deg':   [  0.0,  0.0,  0.0,  0.0,  0.0],
                'max_deg':   [300.0,300.0,300.0,300.0,150.0],

                'control_hz': 30.0,
                'state_hz':   20.0,
                'diag_hz':     1.0,

                'max_speed_deg_s': [60.0,60.0,60.0,60.0,90.0],
                'temp_warn_c': 70.0,
                'volt_warn_v': 10.5,
            }]
        )
    ])

