# 🤖 ROS2 自主导航与视觉抓取机器人系统

> 📚 **南方科技大学 EE211  课程项目**
> 
> 课程主页：[EE211 2025 Fall](https://rpai-lab.github.io/EE211-25Fall/)
基于 ROS2 的移动机器人自主导航与视觉识别抓取系统，集成了 TurtleBot4 底盘、YOLO 目标检测、ArUco 标记识别、机械臂控制等功能。

## ✨ 功能特点

- 🚗 **自主导航**：基于 Nav2 的多点导航与路径规划
- 🎯 **视觉检测**：YOLOv10 实时目标检测（红绿灯、Stop 标志等）
- 📷 **ArUco 识别**：精确的 ArUco 标记定位与姿态估计
- 🦾 **机械臂控制**：自动抓取与放置操作
- 🎮 **云台控制**：Pan-Tilt 云台视觉跟踪
- 🔄 **PID 校准**：X/Y 轴精确位置校准

## 🛠️ 系统要求

### 硬件要求
- TurtleBot4 / iQR-TB4 移动机器人底盘
- Intel RealSense 深度相机
- 机械臂（支持 AX-12A 舵机）
- Pan-Tilt 云台

### 软件依赖
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- OpenCV 4.x
- YOLOv10

## 📦 安装

### 1. 克隆仓库

```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/ros2_ws.git
cd ros2_ws
```

### 2. 安装依赖

```bash
# 安装 ROS2 依赖
rosdep install --from-paths src --ignore-src -r -y

# 安装 Python 依赖
pip3 install ultralytics opencv-python numpy
```

### 3. 编译工作空间

```bash
colcon build --symlink-install
source install/setup.bash
```

## 🚀 快速开始

### 启动顺序（共 6 个终端）

#### 终端 1：机器人基础启动
```bash
ros2 launch iqr_tb4_bringup bringup.launch.py
```

#### 终端 2：定位系统
```bash
cd ~/ros2_ws/src/iqr_tb4_ros/iqr_tb4_navigation/maps/
ros2 launch turtlebot4_navigation localization.launch.py map:=map.yaml
```

#### 终端 3：导航系统
```bash
ros2 launch turtlebot4_navigation nav2.launch.py
```

#### 终端 4：可视化（RViz）
```bash
ros2 launch turtlebot4_viz view_robot.launch.py
```

#### 终端 5：YOLO 视觉检测节点
```bash
cd ~/ros2_ws
python3 src/yolov10_inference/yolov10_inference/yolo_rgs.py --ros-args -p confidence_threshold:=0.08
```

#### 终端 6：导航主节点
```bash
cd ~/ros2_ws
python3 src/waypoint_publisher.py
```

## 📋 任务流程

```
起始位置 ──► 点1 ──► 点2 ──► 点3 ──► 点4 ──► 点5
              │              │       │
              ▼              ▼       ▼
          红绿灯检测      抓取流程  Stop检测
                                    放置流程
```

### 详细流程

| 阶段 | 起点 | 终点 | 任务描述 |
|------|------|------|----------|
| 1 | 起始位置 | 点1 | 导航中检测红绿灯（红灯暂停/绿灯通行）|
| 2 | 点1 | 点2 | 正常导航 |
| 3 | 点2 | 点3 | 正常导航 |
| 4 | 点3 | - | **抓取流程**：打开夹爪 → 调整云台 → 机械臂运动 → ArUco识别 → PID校准 → 关闭夹爪 |
| 5 | 点3 | 点4 | 导航中检测 Stop 标志（检测到则暂停等待）|
| 6 | 点4 | - | **放置流程**：打开夹爪放置物品 |
| 7 | 点4 | 点5 | 返回导航 |

## ⚙️ 参数配置

### YOLO 置信度阈值

```bash
# 默认值 0.08
python3 yolo_rgs.py --ros-args -p confidence_threshold:=0.08
```

| 阈值 | 效果 |
|------|------|
| 0.05 | 更敏感，可能误检 |
| 0.08 | 平衡（推荐）|
| 0.15 | 更精确，可能漏检 |

### 检测确认帧数

在 `waypoint_publisher.py` 中设置：
- 默认值：`1`（检测到 1 次即触发）

## 🔧 调试工具

### 视觉检测可视化

```bash
python3 src/vision_detection_viewer.py
```

### 查看话题列表

```bash
ros2 topic list
```

### 查看检测结果

```bash
ros2 topic echo /yolo/detections
```

## 📁 项目结构

```
ros2_ws/
├── src/
│   ├── iqr_tb4_ros/           # TB4 机器人配置
│   ├── yolov10_inference/     # YOLO 目标检测
│   ├── ros2_aruco_realsense/  # ArUco 识别
│   ├── pan_tilt_ros/          # 云台控制
│   ├── ax12a_arm_controller/  # 机械臂控制
│   ├── waypoint_publisher.py  # 导航主节点
│   ├── X_PID.py               # X轴 PID 控制
│   └── Y_PID.py               # Y轴 PID 控制
├── install/                   # 编译安装目录
├── build/                     # 编译构建目录
└── log/                       # 日志目录
```

## ⚠️ 注意事项

1. 确保所有终端都在 `~/ros2_ws` 目录下运行
2. 启动顺序很重要：先启动基础系统（终端 1-4），再启动视觉检测（终端 5），最后启动导航主节点（终端 6）
3. 确保地图文件路径正确
4. 如果 YOLO 检测不到目标，可调整 `confidence_threshold` 参数
5. 如果导航暂停不工作，检查日志中的检测计数信息

## 🔗 远程连接机器人

详细教程请参考：[Remote Connection Guide](https://rpai-lab.github.io/EE211-25Fall/assets/project/remote_connection/)

### NoMachine 远程桌面

```bash
# 安装 NoMachine
sudo dpkg -i <pkg_name>.deb

# 检查网络连通性
ping <robot_ip>

# 如果连接黑屏，重启 NX 服务
sudo /etc/NX/nxserver --restart
```

### SSH 连接

```bash
ssh <username>@<robot_ip>
# 例如: ssh tony@192.168.1.1
```

### VSCode Remote SSH

推荐使用 VSCode 的 Remote SSH 插件，获得与本地相同的代码编辑体验。

## 📚 参考资料

- [EE211 2025 Fall 课程主页](https://rpai-lab.github.io/EE211-25Fall/)
- [远程连接指南](https://rpai-lab.github.io/EE211-25Fall/assets/project/remote_connection/)
- [ROS2 Humble 官方文档](https://docs.ros.org/en/humble/)
- [Nav2 导航框架](https://navigation.ros.org/)
- [YOLOv10 Ultralytics](https://github.com/ultralytics/ultralytics)

## 📄 许可证

MIT License

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📞 联系方式

如有问题，请提交 Issue 或联系项目维护者。

---

⭐ 如果这个项目对您有帮助，请给个 Star！

