# 白色小物块检测节点

## 功能概述
这个节点专门用于检测白色小方块物块，并计算其在3D空间中的精确位姿。基于Realsense相机的RGB-D数据，使用计算机视觉技术实现。

## 核心特性
- **白色检测**：使用HSV颜色空间精确识别白色物块
- **形状过滤**：通过圆形度和矩形度过滤非矩形物体
- **3D位姿计算**：使用PnP算法计算物块的精确3D位置和姿态
- **深度集成**：利用深度信息提高Z轴精度
- **TF发布**：同时发布PoseStamped消息和TF变换

## 文件结构
```
src/
├── white_block_detector.py      # 主检测节点
├── white_block_detector.launch.py  # 启动文件
├── white_block_params.yaml      # 参数配置文件
└── README_white_block_detector.md   # 本说明文档
```

## 使用方法

### 1. 启动检测节点
```bash
# 使用启动文件
ros2 launch src/white_block_detector.launch.py

# 或者单独运行节点
ros2 run white_block_detector white_block_detector --ros-args --params-file src/white_block_params.yaml
```

### 2. 查看检测结果
```bash
# 查看发布的位姿
ros2 topic echo /white_block_pose

# 查看TF变换
ros2 run tf2_tools view_frames
```

### 3. RViz可视化
```bash
# 启动RViz查看物块位置
ros2 run rviz2 rviz2
# 在RViz中添加TF显示，查看white_block坐标系
```

## 参数配置

### 颜色检测参数
- `white_lower_hsv`: [0, 0, 180] - 白色HSV下限
- `white_upper_hsv`: [180, 30, 255] - 白色HSV上限

### 形状过滤参数
- `min_contour_area`: 100 - 最小轮廓面积（像素）
- `max_contour_area`: 5000 - 最大轮廓面积（像素）
- `min_circularity`: 0.5 - 最小圆形度（0-1）
- `min_solidity`: 0.8 - 最小矩形度（0-1）

### 物块尺寸参数
- `block_width`: 0.03 - 物块宽度（米）
- `block_length`: 0.03 - 物块长度（米）
- `block_height`: 0.02 - 物块高度（米）

## 话题和服务

### 发布的话题
- `/white_block_pose` (geometry_msgs/PoseStamped) - 物块3D位姿
- `/tf` - TF变换（white_block -> camera_color_optical_frame）

### 订阅的话题
- `/camera/color/image_raw` - RGB图像
- `/camera/aligned_depth_to_color/image_raw` - 对齐的深度图像
- `/camera/color/camera_info` - 相机内参

## 调试和故障排除

### 常见问题
1. **检测不到物块**：
   - 检查HSV阈值是否适合当前光照条件
   - 调整`min_contour_area`和`max_contour_area`参数
   - 确保物块表面干净，无反光

2. **位姿计算不准确**：
   - 检查物块实际尺寸参数是否正确
   - 确保相机内参正确加载
   - 检查深度图像质量

3. **TF变换不显示**：
   - 确认`publish_tf`参数为true
   - 检查相机坐标系名称是否正确

### 调试工具
```bash
# 查看节点参数
ros2 param list /white_block_detector
ros2 param get /white_block_detector white_lower_hsv

# 动态调整参数
ros2 param set /white_block_detector min_contour_area 200

# 查看节点日志
ros2 node info /white_block_detector
```

## 性能优化建议
1. 根据实际场景调整HSV阈值
2. 适当调整轮廓面积范围以减少误检
3. 在良好光照条件下使用以获得最佳效果
4. 定期校准相机内参以保证精度

## 扩展功能
- 支持多物块同时检测
- 添加颜色校准功能
- 集成机器学习提高检测精度
- 支持不同形状物块检测
