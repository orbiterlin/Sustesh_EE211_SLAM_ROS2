#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, time, threading
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from control_msgs.msg import GripperCommand  # 夹爪标准消息

# Dynamixel SDK（apt: ros-humble-dynamixel-sdk）
from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS
)

# ===== AX-12A（协议1.0）控制表地址 =====
ADDR_TORQUE_ENABLE        = 24   # 1B
ADDR_GOAL_POSITION        = 30   # 2B
ADDR_PRESENT_POSITION     = 36   # 2B
ADDR_TORQUE_LIMIT         = 34   # 2B（RAM：运行期扭矩上限）
ADDR_PRESENT_VOLTAGE      = 42   # 1B（0.1V）
ADDR_PRESENT_TEMPERATURE  = 43   # 1B（℃）
ADDR_MAX_TORQUE           = 14   # 2B（EEPROM：出厂扭矩上限）

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

RESOLUTION   = 1023            # 位置刻度
DEGREE_RANGE = 300.0           # 角度范围（度）

def rad2deg(rad: float) -> float:
    """将弧度转换为角度（度）。"""
    return rad * 180.0 / math.pi

def deg_to_pos(deg: float) -> int:
    """将角度(度)转换为舵机刻度(0..1023)，并夹紧在 0..300°。"""
    d = max(0.0, min(DEGREE_RANGE, float(deg)))
    return int(round(d / DEGREE_RANGE * RESOLUTION))

def pos_to_deg(pos: int) -> float:
    """将舵机刻度(0..1023)转换为角度(度)。"""
    p = max(0, min(RESOLUTION, int(pos)))
    return p / RESOLUTION * DEGREE_RANGE


class AX12AArmNode(Node):
    """四自由度机械臂 + 独立夹爪驱动节点（AX-12A）。
    - 4 关节轨迹控制：/ax12a_arm/trajectory  (trajectory_msgs/JointTrajectory)
    - 夹爪控制：/ax12a_arm/gripper_cmd       (control_msgs/GripperCommand; position: 0..100，0=闭合/100=张开)
    - 状态发布：/joint_states（含4关节+夹爪）
    - 诊断发布：/diagnostics（电压/温度）
    """

    def __init__(self):
        """构造：读取参数、打开串口、限扭矩、扭矩使能、创建话题/服务/定时器、启动线程与夹爪初始化。"""
        super().__init__('ax12a_arm_node')

        # —— 参数（可在 launch 覆盖） ——
        self.declare_parameters('', [
            # 串口与协议
            ('port', '/dev/ttyUSB0'),
            ('baudrate', 1000000),
            ('protocol', 1.0),

            # 关节 ID（4个）与夹爪 ID（1个）
            ('ids_arm', [1, 2, 3, 4]),
            ('id_gripper', 5),

            # 关节名称（4个）与夹爪名称（1个）
            ('joint_names_arm', ['joint1','joint2','joint3','joint4']),
            ('joint_name_gripper', 'gripper'),

            # 关节/夹爪映射：servo_deg = zero_deg + direction * rad2deg(joint_rad)
            # 注意：数组长度为5（前4=关节，第5=夹爪）
            ('zero_deg',  [150.0,150.0,150.0,150.0,  0.0]),   # 夹爪默认零点=闭合角
            ('direction', [  1.0,-1.0,  1.0,-1.0,    1.0]),   # 夹爪正方向=张开
            ('min_deg',   [  0.0,  0.0,  0.0,  0.0,  0.0]),   # 夹爪最小=0°
            ('max_deg',   [300.0,300.0,300.0,300.0,150.0]),   # 夹爪最大=150°

            # 控制/状态/诊断频率
            ('control_hz', 60.0),
            ('state_hz',   20.0),
            ('diag_hz',     1.0),

            # 速度限幅（度/秒）：前4为关节，第5为夹爪
            ('max_speed_deg_s', [60.0, 60.0, 60.0, 60.0, 180.0]),

            # 诊断阈值
            ('temp_warn_c', 70.0),     # ℃（≥则 WARN）
            ('volt_warn_v', 10.5),     # V（< 则 WARN）

            # 夹爪 effort → 速度缩放（max_effort: 0..1 → 速度比例 [effort_min_scale, 1.0]；<=0 忽略）
            ('use_effort_scale', False),
            ('effort_min_scale', 0.1),

            # 启动夹爪初始化：张到最大（不使用安全边距/缓行）
            ('gripper_startup_init', True),
            ('gripper_startup_pct',  100.0),  # 0..100

            # 夹爪到位容差与硬上限
            ('gripper_goal_tolerance_deg', 1.0),  # 到位判定阈值（度）
            ('gripper_upper_deg_cap', 150.0),     # 二次裁剪硬上限（防止越过 150° 危险半圈）

            # 扭矩限幅（0..1），并可选写入 EEPROM 的 Max Torque（谨慎）
            ('torque_limit_ratio', 0.8),
            ('write_max_torque_eeprom', False),
        ])

        # —— 读取参数到成员变量 ——
        self.port_name  = self.get_parameter('port').value
        self.baudrate   = int(self.get_parameter('baudrate').value)
        self.protocol   = float(self.get_parameter('protocol').value)

        self.ids_arm: List[int] = list(self.get_parameter('ids_arm').value)
        self.id_grip: int       = int(self.get_parameter('id_gripper').value)

        self.jn_arm: List[str]  = list(self.get_parameter('joint_names_arm').value)
        self.jn_grip: str       = self.get_parameter('joint_name_gripper').value

        self.zero_deg  = [float(x) for x in self.get_parameter('zero_deg').value]
        self.direction = [float(x) for x in self.get_parameter('direction').value]
        self.min_deg   = [float(x) for x in self.get_parameter('min_deg').value]
        self.max_deg   = [float(x) for x in self.get_parameter('max_deg').value]
        self.max_speed_deg_s = [float(x) for x in self.get_parameter('max_speed_deg_s').value]

        self.control_dt = 1.0 / max(1e-3, float(self.get_parameter('control_hz').value))
        self.state_dt   = 1.0 / max(1e-3, float(self.get_parameter('state_hz').value))
        self.diag_dt    = 1.0 / max(1e-3, float(self.get_parameter('diag_hz').value))
        self.temp_warn_c = float(self.get_parameter('temp_warn_c').value)
        self.volt_warn_v = float(self.get_parameter('volt_warn_v').value)

        self.use_effort_scale = bool(self.get_parameter('use_effort_scale').value)
        self.effort_min_scale = float(self.get_parameter('effort_min_scale').value)

        self.gripper_startup_init = bool(self.get_parameter('gripper_startup_init').value)
        self.gripper_startup_pct  = float(self.get_parameter('gripper_startup_pct').value)
        self.gripper_goal_tolerance_deg = float(self.get_parameter('gripper_goal_tolerance_deg').value)
        self.gripper_upper_deg_cap = float(self.get_parameter('gripper_upper_deg_cap').value)

        self.torque_limit_ratio = float(self.get_parameter('torque_limit_ratio').value)
        self.write_max_torque_eeprom = bool(self.get_parameter('write_max_torque_eeprom').value)

        assert len(self.ids_arm) == 4 and len(self.jn_arm) == 4, "机械臂应为 4 自由度"
        for arr in (self.zero_deg, self.direction, self.min_deg, self.max_deg, self.max_speed_deg_s):
            assert len(arr) == 5, "映射/限幅参数数组长度必须为 5（含夹爪）"

        # —— 打开串口并设置波特率 ——
        self.port   = PortHandler(self.port_name)
        self.packet = PacketHandler(self.protocol)
        if not self.port.openPort():
            raise RuntimeError(f'无法打开串口: {self.port_name}')
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f'无法设置波特率: {self.baudrate}')

        # —— 互斥锁：串口与SyncWrite串行化，避免并发冲突 ——
        self._port_lock = threading.Lock()
        self._sw_lock   = threading.Lock()

        # —— 启动限扭矩（先设置上限，再开启扭矩） ——
        self._apply_torque_limit()

        # —— 扭矩使能（4关节 + 夹爪） ——
        for dxl_id in (self.ids_arm + [self.id_grip]):
            with self._port_lock:
                res, err = self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if res != COMM_SUCCESS or err != 0:
                raise RuntimeError(f'ID={dxl_id} 扭矩使能失败')

        # —— SyncWrite（Goal Position） ——
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 2)

        # —— 订阅：整臂轨迹（4关节）
        self.sub_traj = self.create_subscription(
            JointTrajectory, '/ax12a_arm/trajectory', self.on_trajectory, 10
        )

        # —— 订阅：夹爪命令（GripperCommand，position: 0..100）
        self.sub_grip = self.create_subscription(
            GripperCommand, '/ax12a_arm/gripper_cmd', self.on_gripper_cmd, 10
        )

        # —— 发布：关节状态与诊断 ——
        self.pub_js = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.timer_js = self.create_timer(self.state_dt, self.publish_joint_state)
        self.timer_diag = self.create_timer(self.diag_dt, self.publish_diagnostics)

        # —— 服务：急停 —— 
        self.srv_estop = self.create_service(Trigger, 'arm_controller/estop', self.on_estop)

        # —— 轨迹执行线程（线性插值 + 速度限幅 + 抢占） —— 
        self._traj_lock = threading.Lock()
        self._active_traj: Optional[JointTrajectory] = None
        self._stop = False
        self._worker = threading.Thread(target=self._traj_loop, daemon=True)
        self._worker.start()

        # —— 速度限幅记忆（度） —— 
        self._last_cmd_degs_arm: Optional[List[float]] = None

        # —— 夹爪内部“目标-跟随”机制 —— 
        self._grip_goal_deg: Optional[float] = None   # 夹爪目标角（度，来自 0..100 映射）
        self._last_cmd_deg_grip: Optional[float] = None
        self._grip_speed_scale: float = 1.0           # 由 max_effort 映射的速度比例
        self.timer_grip = self.create_timer(self.control_dt, self._gripper_step)

        # —— 启动夹爪初始化（张到最大开度，不留边距/缓行） —— 
        if self.gripper_startup_init:
            threading.Thread(target=self._gripper_startup_init, daemon=True).start()

        self.get_logger().info('AX-12A (四自由度 + 夹爪) 驱动已启动!')

    # ======================== 启动限扭矩 ========================

    def _apply_torque_limit(self):
        """启动阶段设置扭矩上限：写 Torque Limit(34)；可选写 EEPROM 的 Max Torque(14)。"""
        ratio = max(0.05, min(1.0, self.torque_limit_ratio))  # 5%..100%
        limit_val = int(round(ratio * RESOLUTION))            # 0..1023
        for dxl_id in (self.ids_arm + [self.id_grip]):
            with self._port_lock:
                # 运行期软限（推荐必写）
                self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_TORQUE_LIMIT, limit_val)
                # EEPROM 上限（可选；若你频繁改，不建议反复写 EEPROM）
                if self.write_max_torque_eeprom:
                    self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_MAX_TORQUE, limit_val)

    # ======================== 订阅回调 ========================

    def on_trajectory(self, traj: JointTrajectory):
        """整臂 4 关节 JointTrajectory：抢占旧轨迹，交给线程按 control_dt 插值执行。"""
        if traj.points and any(len(p.positions) != 4 for p in traj.points):
            self.get_logger().error('JointTrajectory 维度应为 4（不包含夹爪）'); return
        with self._traj_lock:
            self._active_traj = traj

    def on_gripper_cmd(self, msg: GripperCommand):
        """夹爪控制（0..100% → 舵机角度）：线性映射，无边距/缓行；仅可选使用 max_effort 作为速度比例。"""
        # —— 1) 百分比到角度（线性 + 硬上限二次裁剪） ——
        target_deg = self._percent_to_gripper_deg(msg.position)

        # —— 2) 速度缩放：仅由 max_effort（0..1）决定；若禁用 use_effort_scale 或未给/<=0 则=1.0 —— 
        if self.use_effort_scale and (msg.max_effort is not None) and (msg.max_effort > 0.0):
            self._grip_speed_scale = max(self.effort_min_scale, min(1.0, float(msg.max_effort)))
        else:
            self._grip_speed_scale = 1.0

        # —— 3) 记录目标，交由 _gripper_step() 跟随 ——
        self._grip_goal_deg = target_deg

    # ======================== 状态发布 ========================

    def publish_joint_state(self):
        """读取 4 关节 + 夹爪 的当前位置，映射为关节弧度并发布 JointState。"""
        names = self.jn_arm + [self.jn_grip]
        positions_rad = []

        # 四个关节
        for i, dxl_id in enumerate(self.ids_arm):
            with self._port_lock:
                pos, res, err = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
            if res != COMM_SUCCESS or err != 0:
                self.get_logger().warn(f'读取关节 ID={dxl_id} 位置失败'); return
            deg = pos_to_deg(int(pos))
            sign = self.direction[i] if abs(self.direction[i])>1e-9 else 1.0
            rad = math.radians((deg - self.zero_deg[i]) / sign)
            positions_rad.append(rad)

        # 夹爪
        with self._port_lock:
            pos, res, err = self.packet.read2ByteTxRx(self.port, self.id_grip, ADDR_PRESENT_POSITION)
        if res != COMM_SUCCESS or err != 0:
            self.get_logger().warn(f'读取夹爪 ID={self.id_grip} 位置失败'); return
        deg = pos_to_deg(int(pos))
        sign = self.direction[4] if abs(self.direction[4])>1e-9 else 1.0
        rad = math.radians((deg - self.zero_deg[4]) / sign)
        positions_rad.append(rad)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = names
        js.position = positions_rad
        self.pub_js.publish(js)

    # ======================== 诊断发布 ========================

    def publish_diagnostics(self):
        """读取 5 个电机温度/电压，发布 DiagnosticArray；超阈值置 WARN。"""
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        all_ids = self.ids_arm + [self.id_grip]
        for dxl_id in all_ids:
            status = DiagnosticStatus()
            status.name = f'AX12A_ID{dxl_id}'
            status.hardware_id = str(dxl_id)

            with self._port_lock:
                v_raw, rc_v, err_v = self.packet.read1ByteTxRx(self.port, dxl_id, ADDR_PRESENT_VOLTAGE)
                t_raw, rc_t, err_t = self.packet.read1ByteTxRx(self.port, dxl_id, ADDR_PRESENT_TEMPERATURE)

            voltage = None if rc_v != COMM_SUCCESS or err_v != 0 else (v_raw * 0.1)
            temp_c  = None if rc_t != COMM_SUCCESS or err_t != 0 else float(t_raw)

            kv = []
            level = DiagnosticStatus.OK
            msg = 'OK'
            if voltage is None:
                kv.append(KeyValue(key='voltage', value='read_fail'))
                level = max(level, DiagnosticStatus.WARN); msg='voltage read fail'
            else:
                kv.append(KeyValue(key='voltage_V', value=f'{voltage:.2f}'))
                if voltage < self.volt_warn_v:
                    level = max(level, DiagnosticStatus.WARN); msg='low voltage'

            if temp_c is None:
                kv.append(KeyValue(key='temperature', value='read_fail'))
                level = max(level, DiagnosticStatus.WARN)
                msg = 'temp read fail' if msg=='OK' else (msg+', temp read fail')
            else:
                kv.append(KeyValue(key='temperature_C', value=f'{temp_c:.1f}'))
                if temp_c >= self.temp_warn_c:
                    level = max(level, DiagnosticStatus.WARN)
                    msg = 'overheat' if msg=='OK' else (msg+', overheat')

            status.level = level
            status.message = msg
            status.values = kv
            array.status.append(status)

        self.pub_diag.publish(array)

    # ======================== 服务：急停 ========================

    def on_estop(self, req, resp):
        """急停：关闭 4 关节 + 夹爪 的扭矩。"""
        ok = True
        for dxl_id in (self.ids_arm + [self.id_grip]):
            with self._port_lock:
                res, err = self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if res != COMM_SUCCESS or err != 0:
                ok = False
        resp.success = ok
        resp.message = 'torque disabled' if ok else 'some joints failed'
        return resp

    # ======================== 轨迹执行线程（臂：4 关节） ========================

    def _traj_loop(self):
        """执行当前 4 关节轨迹：支持抢占；第一段从“当前姿态”平滑插到第一点；逐周期速度限幅。"""
        clock = Clock()
        last_tick = time.monotonic()
        while not self._stop:
            with self._traj_lock:
                traj = self._active_traj
            if not traj or not traj.points:
                time.sleep(0.01); continue

            # 维度校验（每点 positions 为 4 维）
            if any(len(p.positions) != 4 for p in traj.points):
                self.get_logger().error('JointTrajectory 维度应为 4'); 
                with self._traj_lock: self._active_traj = None
                continue

            # 参考时刻与当前姿态（4 关节，rad）
            t0_clock = clock.now()
            cur_rad4 = self._read_current_joint_radians_arm()

            # 初始化“上次下发”（度）
            if self._last_cmd_degs_arm is None:
                self._last_cmd_degs_arm = [ self._jointrad_to_servodeg(i, cur_rad4[i]) for i in range(4) ]

            pts = list(traj.points)

            # 逐段执行
            for k in range(len(pts)):
                with self._traj_lock:
                    if traj is not self._active_traj: break

                if k == 0:
                    p_prev = cur_rad4; t_prev = 0.0
                else:
                    p_prev = pts[k-1].positions
                    t_prev = pts[k-1].time_from_start.sec + pts[k-1].time_from_start.nanosec*1e-9
                p_curr = pts[k].positions
                t_curr = pts[k].time_from_start.sec + pts[k].time_from_start.nanosec*1e-9

                if abs(t_curr - t_prev) < 1e-6:
                    servo_degs = [ self._jointrad_to_servodeg(i, p_curr[i]) for i in range(4) ]
                    self._syncwrite_degs_arm_limited(servo_degs)
                    continue

                while True:
                    with self._traj_lock:
                        if traj is not self._active_traj: break
                    elapsed = (clock.now() - t0_clock).nanoseconds * 1e-9
                    if elapsed >= t_curr: break

                    s = max(t_prev, elapsed)
                    alpha = (s - t_prev) / (t_curr - t_prev)
                    cmd = [ (1.0-alpha)*p_prev[i] + alpha*p_curr[i] for i in range(4) ]
                    servo_degs = [ self._jointrad_to_servodeg(i, cmd[i]) for i in range(4) ]
                    self._syncwrite_degs_arm_limited(servo_degs)

                    dt = self.control_dt - (time.monotonic() - last_tick)
                    if dt > 0: time.sleep(dt)
                    last_tick = time.monotonic()

            with self._traj_lock:
                if traj is self._active_traj: self._active_traj = None

    # ======================== 夹爪跟随定时器 ========================

    def _gripper_step(self):
        """定时器：若存在夹爪目标角（度），则按速度限幅逐步靠近并下发；处理竞态并避免 None 运算。"""
        idx = 4

        # —— 安全读取当前目标（避免中途被置 None 导致竞态） ——
        goal = self._grip_goal_deg
        if goal is None:
            return

        # —— 本周期最大步长（受 max_effort 速度比例影响） ——
        max_step = self.max_speed_deg_s[idx] * self.control_dt * max(0.1, min(1.0, self._grip_speed_scale))

        # —— 首次：读取当前角度作为“上次值”，避免大步瞬移 ——
        if self._last_cmd_deg_grip is None:
            with self._port_lock:
                pos, rc, err = self.packet.read2ByteTxRx(self.port, self.id_grip, ADDR_PRESENT_POSITION)
            cur_deg = self.zero_deg[idx] if (rc != COMM_SUCCESS or err != 0) else pos_to_deg(int(pos))
            self._last_cmd_deg_grip = self._clamp_deg(idx, cur_deg)

        prev = self._last_cmd_deg_grip
        tgt  = goal  # 使用本地快照，避免中途被修改

        # —— 限速步进 ——
        delta = tgt - prev
        if   delta >  max_step: cmd = prev + max_step
        elif delta < -max_step: cmd = prev - max_step
        else:                   cmd = tgt

        # —— 记录并下发 —— 
        self._last_cmd_deg_grip = cmd
        self._syncwrite_degs([(self.id_grip, cmd)])

        # —— 判定到位：再次读取当前目标快照，若仍存在且接近则清空 —— 
        tol = self.gripper_goal_tolerance_deg
        goal_now = self._grip_goal_deg
        if (goal_now is not None) and (abs(goal_now - cmd) <= tol):
            self._grip_goal_deg = None

    # ======================== 工具：百分比线性映射 ========================

    def _percent_to_gripper_deg(self, pct: float) -> float:
        """将 0..100 的开合百分比线性映射为舵机角度（度），不使用安全边距/缓行；最后与硬上限裁剪。"""
        idx = 4
        v = max(0.0, min(100.0, float(pct)))     # 0..100
        deg_min = self.min_deg[idx]              # 通常=0°
        # 二次裁剪：max_deg 与 硬上限 取 min，杜绝越过 150° 危险半圈
        deg_max = min(self.max_deg[idx], self.gripper_upper_deg_cap)
        deg = deg_min + (v / 100.0) * (deg_max - deg_min)
        return self._clamp_deg(idx, deg)

    # ======================== 辅助：读取当前姿态（臂） ========================

    def _read_current_joint_radians_arm(self) -> List[float]:
        """读取 4 关节当前值（rad），不含夹爪。"""
        res_rad = []
        for i, dxl_id in enumerate(self.ids_arm):
            with self._port_lock:
                pos, rc, err = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
            if rc != COMM_SUCCESS or err != 0:
                deg = self.zero_deg[i]
            else:
                deg = pos_to_deg(int(pos))
            sign = self.direction[i] if abs(self.direction[i])>1e-9 else 1.0
            rad = math.radians((deg - self.zero_deg[i]) / sign)
            res_rad.append(rad)
        return res_rad

    # ======================== 映射/夹紧/限速/下发（臂） ========================

    def _jointrad_to_servodeg(self, i: int, joint_rad: float) -> float:
        """将关节弧度映射为舵机角度度，并按物理限位夹紧。i=0..3 对应四个关节；i=4 对应夹爪。"""
        deg = self.zero_deg[i] + self.direction[i] * rad2deg(joint_rad)
        return self._clamp_deg(i, deg)

    def _clamp_deg(self, i: int, deg: float) -> float:
        """夹紧舵机角度（度）到 [min_deg[i], max_deg[i]]。"""
        return max(self.min_deg[i], min(self.max_deg[i], float(deg)))

    def _speed_limit_step_array(self, idxs: List[int], prev: List[float], target: List[float]) -> List[float]:
        """数组版速度限幅：按每关节最大角速度限制每周期最大步长。"""
        limited = []
        for j, i in enumerate(idxs):
            prev_i = prev[j]
            tgt_i  = target[j]
            max_step = self.max_speed_deg_s[i] * self.control_dt
            delta = tgt_i - prev_i
            if   delta >  max_step: tgt_i = prev_i + max_step
            elif delta < -max_step: tgt_i = prev_i - max_step
            limited.append(tgt_i)
        return limited

    def _syncwrite_degs(self, id_deg_pairs: List[Tuple[int, float]]):
        """底层 SyncWrite：按 (id, deg) 列表一次性写入；带互斥与重试。"""
        with self._sw_lock:
            self.sync_write.clearParam()
            for dxl_id, d in id_deg_pairs:
                pos = deg_to_pos(d)
                param = [pos & 0xFF, (pos >> 8) & 0xFF]
                assert self.sync_write.addParam(dxl_id, bytes(param)), "SyncWrite 参数添加失败"

            attempts = 0
            while attempts < 3:
                with self._port_lock:
                    res = self.sync_write.txPacket()
                if res == COMM_SUCCESS:
                    return
                attempts += 1
                try:
                    err_str = self.packet.getTxRxResult(res)
                except Exception:
                    err_str = f'code={res}'
                self.get_logger().warn(f'SyncWrite 发送失败，{err_str}（第{attempts}次）')
                time.sleep(0.002 * attempts)

    def _syncwrite_degs_arm_limited(self, servo_degs_arm: List[float]):
        """对 4 关节目标角（度）做速度限幅并一次性下发。"""
        idxs = [0,1,2,3]
        if self._last_cmd_degs_arm is None:
            limited = servo_degs_arm
        else:
            limited = self._speed_limit_step_array(idxs, self._last_cmd_degs_arm, servo_degs_arm)
        self._last_cmd_degs_arm = list(limited)
        pairs = list(zip(self.ids_arm, limited))
        self._syncwrite_degs(pairs)

    # ======================== 启动：夹爪初始化 ========================

    def _gripper_startup_init(self):
        """启动时夹爪初始化：张到最大（线性映射，无边距/缓行），等待到位并对齐内部状态。"""
        try:
            target_deg = self._percent_to_gripper_deg(self.gripper_startup_pct)

            # 初始化：先清空历史，允许第一次读当前位置对齐
            self._grip_speed_scale = 1.0
            self._last_cmd_deg_grip = None
            self._grip_goal_deg = target_deg

            # 等待若干周期到位或超时
            t0 = time.monotonic()
            timeout = 2.5  # 秒
            tol = 2.0      # 度
            while time.monotonic() - t0 < timeout:
                time.sleep(self.control_dt)
                with self._port_lock:
                    pos, rc, err = self.packet.read2ByteTxRx(self.port, self.id_grip, ADDR_PRESENT_POSITION)
                if rc == COMM_SUCCESS and err == 0:
                    cur_deg = pos_to_deg(int(pos))
                    if abs(cur_deg - target_deg) <= tol:
                        break

            # 对齐内部状态
            self._last_cmd_deg_grip = target_deg
            self._grip_goal_deg = None
            self.get_logger().info('夹爪已初始化至最大开度!')
        except Exception as e:
            self.get_logger().warn(f'夹爪初始化失败：{e}')

    # ======================== 析构收尾 ========================

    def destroy_node(self):
        """停止线程、关闭扭矩与串口，确保硬件安全。"""
        try:
            self._stop = True
            self._worker.join(timeout=1.0)
        except Exception:
            pass
        try:
            for dxl_id in (self.ids_arm + [self.id_grip]):
                with self._port_lock:
                    self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        except Exception:
            pass
        try:
            self.port.closePort()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """入口：初始化 ROS 2、启动节点并进入自旋。"""
    rclpy.init(args=args)
    node = AX12AArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
