#!/usr/bin/env python3
import subprocess
import os
import sys
import time

def open_terminal_and_execute(cmd, title):
    """
    新建终端并执行指定命令
    :param cmd: 要执行的命令字符串
    :param title: 终端窗口标题（便于区分）
    """
    # 适配Ubuntu GNOME终端（其他终端可调整命令，如xterm）
    terminal_cmd = [
        "gnome-terminal",
        "--title", title,
        "--", "bash", "-c", f"{cmd}; exec bash"  # 执行完命令后保持终端打开
    ]
    try:
        subprocess.Popen(terminal_cmd)
        print(f"✅ 新建终端[{title}]并执行命令：{cmd}")
    except Exception as e:
        print(f"❌ 终端[{title}]启动失败：{e}")
        sys.exit(1)

def main():
    # 定义3个终端要执行的命令（按你的需求）
    cmd0 = "ros2 topic pub -1 /ax12a_arm/gripper_cmd control_msgs/msg/GripperCommand '{position: 100.0, max_effort: 0.5}'"
    cmd1 = "ros2 topic pub /pan_tilt_cmd_deg pan_tilt_msgs/msg/PanTiltCmdDeg '{yaw: 0.0, pitch: 25.0, speed: 5}'"
    cmd2 = "ros2 topic pub -1 /ax12a_arm/trajectory trajectory_msgs/JointTrajectory '{ joint_names: ['joint1','joint2','joint3','joint4'], points: [ { positions: [0.0, 0.0, 1.05, 0.95], time_from_start: {sec: 5.0} } ] }'"
    cmd3 = "cd ~/ros2_ws && python3 ./src/aruco_pose_estimator.py"
    cmd4 = "ros2 topic pub -1 /ax12a_arm/trajectory trajectory_msgs/JointTrajectory '{ joint_names: ['joint1','joint2','joint3','joint4'], points: [ { positions: [0.0, -1.5, 1.05, 0.95], time_from_start: {sec: 5.0} } ] }'"
    cmd5 = "ros2 topic pub -1 /ax12a_arm/gripper_cmd control_msgs/msg/GripperCommand '{position: 25.0, max_effort: 0.5}'"
    cmd6 = "cd ~/ros2_ws && python3 ./src/X_PID.py"
    cmd7 = "cd ~/ros2_ws && python3 ./src/Y_PID.py"
    # 切换到用户主目录（避免路径问题）
    os.chdir(os.path.expanduser("~"))

    # 逐个启动终端执行命令
    open_terminal_and_execute(cmd0, "夹爪-0")
    open_terminal_and_execute(cmd1, "云台控制")
    open_terminal_and_execute(cmd2, "机械臂运动-1")
    open_terminal_and_execute(cmd3, "ArUco识别")
    time.sleep(7)
    open_terminal_and_execute(cmd4, "机械臂运动-2")
    open_terminal_and_execute(cmd6, "校准x轴")
    open_terminal_and_execute(cmd7, "校准y轴")

    time.sleep(20)
    open_terminal_and_execute(cmd5, "夹爪-1")

    print("\n🎉 所有终端已启动，抓取流程开始！")
    print("提示：按Ctrl+C可终止对应终端的命令，关闭终端可退出。")

if __name__ == "__main__":
    # 检查是否为Ubuntu系统（可选）
    if not os.path.exists("/usr/bin/gnome-terminal"):
        print("❌ 未找到gnome-terminal，请安装：sudo apt install gnome-terminal")
        sys.exit(1)
    
    # 检查ROS 2环境（可选）
    if "ROS_DISTRO" not in os.environ or os.environ["ROS_DISTRO"] != "humble":
        print("⚠️ 未检测到ROS 2 Humble环境，建议先执行：source /opt/ros/humble/setup.bash")
    
    main()
