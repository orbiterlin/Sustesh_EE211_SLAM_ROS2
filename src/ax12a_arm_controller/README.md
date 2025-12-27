# AX12A 四自由度机械臂 + 夹爪 ROS2 驱动

基于 **DYNAMIXEL AX-12A（协议 1.0）** 的四自由度机械臂驱动节点，带独立夹爪控制。  

- 轨迹控制：`/ax12a_arm/trajectory`（`trajectory_msgs/JointTrajectory`，**4 关节，不包含夹爪**）  
- 夹爪控制：`/ax12a_arm/gripper_cmd`（`control_msgs/GripperCommand`，`position: 0..100`，0=闭合、100=张开）  
- 状态发布：`/joint_states`（含 4 关节 + 夹爪，单位：弧度）  
- 诊断发布：`/diagnostics`（电压/温度）  



---

## 1. 环境要求

- Ubuntu 22.04

- ROS 2 Humble

- Python 3.10+

- DYNAMIXEL SDK（ROS 发行版包）

  ```bash
  sudo apt install ros-humble-dynamixel-sdk
  ```

> 确保串口权限（将用户加入 `dialout`，或设置 udev 规则）：

```bash
sudo usermod -a -G dialout $USER
# 重新登录生效；或临时：sudo chmod 666 /dev/ttyUSB0
```

---

## 2. 代码放置与编译

```bash
# 假设工作空间 ~/ros2_ws
cd ~/ros2_ws/src
# 包名称：ax12a_arm_controller（已创建）
# 覆盖节点文件
#   ~/ros2_ws/src/ax12a_arm_controller/ax12a_arm_controller/ax12a_arm_node.py

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

依赖（`package.xml`）需包含：

```xml
<exec_depend>control_msgs</exec_depend>
<exec_depend>diagnostic_msgs</exec_depend>
<exec_depend>trajectory_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>dynamixel_sdk</exec_depend>
```

---

## 3. 启动

建议使用自定义 `launch` 文件传参（示例见下），或直接运行节点并用 `ros2 param set`/`--ros-args -p` 传入参数。

```bash
ros2 run ax12a_arm_controller ax12a_arm_node
# 或
ros2 launch ax12a_arm_controller ax12a_arm.launch.py
```

---

## 4. 话题接口

### 4.1 机械臂轨迹（4 关节）

- 话题：`/ax12a_arm/trajectory`
- 类型：`trajectory_msgs/JointTrajectory`
- 约束：`points[*].positions` 必须是 **4 维**（不包含夹爪）

示例（2 秒到位）：

```bash
ros2 topic pub -1 /ax12a_arm/trajectory trajectory_msgs/JointTrajectory "{
  joint_names: ['joint1','joint2','joint3','joint4'],
  points: [ { positions: [0.3, -0.2, 0.5, 0.0], time_from_start: {sec: 2} } ]
}"
```

### 4.2 夹爪（独立控制）

- 话题：`/ax12a_arm/gripper_cmd`
- 类型：`control_msgs/GripperCommand`
- 语义：
  - `position`：**0..100**（0=闭合、100=张开）
  - `max_effort`：可选（0..1）速度比例；若禁用 `use_effort_scale` 或不填/≤0，则按满速

示例：

```bash
# 张开到 100%（满速）
ros2 topic pub -1 /ax12a_arm/gripper_cmd control_msgs/msg/GripperCommand "{position: 100.0}"

# 慢速闭合（仅当 use_effort_scale=True）
ros2 topic pub -1 /ax12a_arm/gripper_cmd control_msgs/msg/GripperCommand "{position: 0.0, max_effort: 0.5}"
```

### 4.3 关节状态

- 话题：`/joint_states`（`sensor_msgs/JointState`）
- `name`：`['joint1','joint2','joint3','joint4','gripper']`
- `position`：弧度

### 4.4 诊断信息

- 话题：`/diagnostics`（`diagnostic_msgs/DiagnosticArray`）
- 包含每个舵机的电压（V）、温度（℃）；低电压或过热发 WARN

### 4.5 急停服务

- 服务：`arm_controller/estop`（`std_srvs/Trigger`）
- 效果：关闭全部扭矩

---

## 5. 关键参数（可在 launch 中覆盖）

| 参数名                       | 类型      |                                  缺省值 | 说明                                         |
| ---------------------------- | --------- | --------------------------------------: | -------------------------------------------- |
| `port`                       | string    |                          `/dev/ttyUSB0` | 串口设备                                     |
| `baudrate`                   | int       |                               `1000000` | 波特率                                       |
| `protocol`                   | float     |                                   `1.0` | DXL 协议版本                                 |
| `ids_arm`                    | int[4]    |                             `[1,2,3,4]` | 4 关节 ID                                    |
| `id_gripper`                 | int       |                                     `5` | 夹爪 ID                                      |
| `joint_names_arm`            | string[4] | `['joint1','joint2','joint3','joint4']` | 关节名                                       |
| `joint_name_gripper`         | string    |                               `gripper` | 夹爪名                                       |
| `zero_deg`                   | float[5]  |                   `[150,150,150,150,0]` | 零点角（度），夹爪默认闭合=0°                |
| `direction`                  | float[5]  |                         `[1,-1,1,-1,1]` | 正负方向；夹爪正向=张开                      |
| `min_deg`                    | float[5]  |                           `[0,0,0,0,0]` | 物理最小角（度）                             |
| `max_deg`                    | float[5]  |                 `[300,300,300,300,150]` | 物理最大角（度）；**夹爪 0..150**            |
| `gripper_upper_deg_cap`      | float     |                                 `150.0` | **夹爪硬上限**二次裁剪（强烈建议保留）       |
| `control_hz`                 | float     |                                  `60.0` | 控制频率                                     |
| `state_hz`                   | float     |                                  `20.0` | 状态发布频率                                 |
| `diag_hz`                    | float     |                                   `1.0` | 诊断发布频率                                 |
| `max_speed_deg_s`            | float[5]  |                     `[60,60,60,60,180]` | 每关节最大角速度（度/秒）                    |
| `use_effort_scale`           | bool      |                                 `False` | 是否使用 `max_effort` 缩放速度               |
| `effort_min_scale`           | float     |                                   `0.1` | 速度下限比例（仅当 `use_effort_scale=True`） |
| `gripper_startup_init`       | bool      |                                  `True` | 启动是否初始化夹爪                           |
| `gripper_startup_pct`        | float     |                                 `100.0` | 启动初始化目标开合百分比                     |
| `gripper_goal_tolerance_deg` | float     |                                   `1.0` | 夹爪到位判定阈值（度）                       |
| `torque_limit_ratio`         | float     |                                   `0.8` | 扭矩上限比例（写 Torque Limit）              |
| `write_max_torque_eeprom`    | bool      |                                 `False` | 是否写入 EEPROM 的 Max Torque（谨慎）        |
| `temp_warn_c`                | float     |                                  `70.0` | 过热 WARN 阈值（℃）                          |
| `volt_warn_v`                | float     |                                  `10.5` | 低电压 WARN 阈值（V）                        |

> **时间轨迹与速度**：给 `JointTrajectory` 的 `time_from_start` 太小（例如 `0.1s`），当 `Δ角度 / 时间` 超过 `max_speed_deg_s` 时，线程会在段时间到点直接结束，看起来“几乎不动”。请给出与 `max_speed_deg_s` 匹配的合理时间，或提升 `max_speed_deg_s`/`control_hz`。

---

## 6. 示例 launch 片段

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ax12a_arm_controller',
            executable='ax12a_arm_node',
            name='ax12a_arm_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 1000000,
                'ids_arm': [1,2,3,4],
                'id_gripper': 5,
                'zero_deg':  [150.0,150.0,150.0,150.0,0.0],
                'direction': [1.0,-1.0,1.0,-1.0,1.0],
                'min_deg':   [0.0,0.0,0.0,0.0,0.0],
                'max_deg':   [300.0,300.0,300.0,300.0,150.0],
                'gripper_upper_deg_cap': 150.0,
                'control_hz': 60.0,
                'max_speed_deg_s': [60.0,60.0,60.0,60.0,180.0],
                'torque_limit_ratio': 0.8,
                'use_effort_scale': False,
                'gripper_startup_init': True,
                'gripper_startup_pct': 100.0,
            }]
        )
    ])
```

---

## 7. 调试与常见问题

- **SyncWrite 发送失败（间歇 WARN）**  
  典型是**总线干扰/供电不足/波特率不稳**。  
  - 电源足够：AX-12A 建议 ≥ 9–12V，≥3A（整臂更高更稳）  
  - 总线接头牢靠，线材不松动  
  - 波特率先用 1Mbps，问题多时可尝试降到 57600/115200 验证

- **夹爪“开到最大”后掉线**  
  这通常是撞极限过流。已在驱动层限定夹爪 **0..150°**，并建议 `torque_limit_ratio` 合理。若机构真实极限更小，请把 `max_deg[4]` 和/或 `gripper_upper_deg_cap` 再调小。

- **`time_from_start` 很小不动**  
  见上文“时间轨迹与速度”。给足够的时间，或提高 `max_speed_deg_s`/`control_hz`。

- **`/joint_states` 的夹爪读数不直观**  
  这是**真实角度（弧度）**。若想显示 0..1 标准化，需要在 URDF 或上层做缩放；或改驱动增加发布缩放（目前保留真实值）。

- **读电压/温度失败**  
  多为瞬时通信失败；若持续失败，检查线材/供电；电压 WARN 默认 <10.5V。若使用 2S 锂电，请将阈值改到 ~7.2V。

---

## 8. 安全提示

- 实机前先**抬空**测试，确认所有关节与夹爪运动方向/范围正确。  
- 初期将 `torque_limit_ratio` 设为 `0.5~0.8`，逐步提高。  
- `max_deg`/`min_deg` 必须与**真实机构**一致；夹爪确保 `max_deg[4] ≤ 150°`。  
- 需要时可随时调用 `arm_controller/estop` 关闭扭矩。





---

## 9. 致谢

- ROBOTIS DYNAMIXEL SDK  
- ROS 2 社区  
