#!/usr/bin/env python3
# waypoint_publisher.py - 集成视觉检测和抓取流程的导航节点

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
import math
import time
import subprocess
import os
import signal


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # 定义途径点
        self.waypoints = [
            (5.65, -0.15, 90.0),  # 点1
            (5.65,  2.0,  90.0),  # 点2
            (5.15,  3.6,  45.0),  # 点3
            (3.3,   3.6, -180.0), # 点4
            (2.15,  0.1,  0.0),   # 点5
        ]
        
        # 导航客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 直接订阅YOLO检测结果（简化流程）
        self.yolo_sub = self.create_subscription(
            String,
            '/yolo_detection_results',
            self.yolo_callback,
            10
        )
        
        # 订阅视觉控制信号（保留兼容性）
        self.vision_control_sub = self.create_subscription(
            String,
            '/vision/nav_control',
            self.vision_control_callback,
            10
        )
        
        # 状态管理
        self.current_index = 0
        self.current_goal_handle = None
        self.current_goal_index = None
        self.vision_control_state = 'RESUME'  # RESUME 或 PAUSE
        self.is_paused_by_vision = False
        
        # 视觉检测进程
        self.vision_coordinator_process = None
        
        # 状态机
        self.state = 'NAVIGATING'  # NAVIGATING, PAUSED_BY_VISION, GRASPING, PLACING
        
        # 直接检测模式（不使用vision_coordinator）
        self.detection_mode = None  # 初始为None，等待启动
        self.detection_state = 'RESUME'  # RESUME 或 PAUSE
        self.stop_count = 0
        self.go_count = 0
        self.no_stop_count = 0
        self.confirm_frames = 1  # 降低到2帧，更快响应
        self.no_stop_threshold = 15  # 降低到5帧，更快恢复
        
        self.get_logger().info('导航节点已初始化')
    
    def yolo_callback(self, msg: String):
        """直接处理YOLO检测结果"""
        if self.detection_mode is None:
            return  # 检测已禁用
        
        text = msg.data.lower()
        
        if self.detection_mode == 'traffic_light':
            # 交通灯模式
            if 'red' in text:
                self.stop_count += 1
                self.go_count = 0
                self.get_logger().info(f'🔴 检测到红灯 (stop_count={self.stop_count}/{self.confirm_frames})')
                if self.stop_count >= self.confirm_frames and self.detection_state == 'RESUME':
                    self.detection_state = 'PAUSE'
                    self.get_logger().info('🛑 确认红灯，暂停导航')
                    self.pause_navigation()
            elif 'green' in text:
                self.go_count += 1
                self.stop_count = 0
                self.get_logger().info(f'🟢 检测到绿灯 (go_count={self.go_count}/{self.confirm_frames})')
                if self.go_count >= self.confirm_frames and self.detection_state == 'PAUSE':
                    self.detection_state = 'RESUME'
                    self.get_logger().info('✅ 确认绿灯，恢复导航')
                    self.resume_navigation()
            else:
                # none情况，轻微衰减stop_count，保持go_count
                if self.detection_state == 'RESUME':
                    self.stop_count = max(0, self.stop_count - 1)
        
        elif self.detection_mode == 'stop_sign':
            # Stop牌子模式
            if 'stop' in text:
                self.stop_count += 1
                self.go_count = 0
                self.no_stop_count = 0
                self.get_logger().info(f'🛑 检测到stop牌子 (stop_count={self.stop_count}/{self.confirm_frames})')
                if self.stop_count >= self.confirm_frames and self.detection_state == 'RESUME':
                    self.detection_state = 'PAUSE'
                    self.get_logger().info('🛑 确认stop牌子，暂停导航')
                    self.pause_navigation()
            else:
                # 没有检测到stop
                if self.detection_state == 'PAUSE':
                    self.no_stop_count += 1
                    self.get_logger().info(f'⚪ 未检测到stop (no_stop_count={self.no_stop_count}/{self.no_stop_threshold})')
                    if self.no_stop_count >= self.no_stop_threshold:
                        self.detection_state = 'RESUME'
                        self.stop_count = 0
                        self.get_logger().info('✅ 确认无stop牌子，恢复导航')
                        self.resume_navigation()
                else:
                    self.stop_count = max(0, self.stop_count - 1)
    
    def vision_control_callback(self, msg: String):
        """处理视觉控制信号（兼容旧接口）"""
        control = msg.data
        self.vision_control_state = control
        
        if control == 'PAUSE' and not self.is_paused_by_vision:
            self.pause_navigation()
        elif control == 'RESUME' and self.is_paused_by_vision:
            self.resume_navigation()
    
    def pause_navigation(self):
        """暂停导航"""
        if self.current_goal_handle is None:
            self.get_logger().warn('⚠️  无法暂停：没有活动的导航目标')
            return
        
        if self.state != 'NAVIGATING':
            self.get_logger().warn(f'⚠️  无法暂停：当前状态是 {self.state}，不是 NAVIGATING')
            return
        
        if self.is_paused_by_vision:
            self.get_logger().debug('已经处于暂停状态')
            return
        
        self.get_logger().info('⏸️  视觉检测要求暂停导航')
        self.is_paused_by_vision = True
        self.state = 'PAUSED_BY_VISION'
        # 取消当前导航目标
        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_goal_callback)
    
    def resume_navigation(self):
        """恢复导航"""
        if not self.is_paused_by_vision:
            self.get_logger().debug('已经处于运行状态')
            return
        
        if self.current_goal_index is None:
            self.get_logger().warn('⚠️  无法恢复：没有目标索引')
            return
        
        self.get_logger().info('▶️  视觉检测允许恢复导航')
        self.is_paused_by_vision = False
        self.state = 'NAVIGATING'
        # 重新发送当前waypoint
        self.send_waypoint(self.current_goal_index)
    
    def cancel_goal_callback(self, future):
        """取消目标回调"""
        cancel_response = future.result()
        if cancel_response.return_code == 1:  # SUCCESS
            self.get_logger().info('✅ 导航目标已取消')
        else:
            self.get_logger().warn(f'⚠️  取消导航目标失败: {cancel_response.return_code}')
    
    def create_pose(self, x, y, yaw_deg):
        """创建位姿"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        yaw_rad = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw_rad / 2)
        pose.pose.orientation.w = math.cos(yaw_rad / 2)
        
        return pose
    
    def start_vision_coordinator(self, mode='traffic_light'):
        """启动视觉检测协调节点"""
        if self.vision_coordinator_process:
            self.stop_vision_coordinator()
        
        try:
            self.get_logger().info(f'🎥 启动视觉检测协调节点，模式: {mode}')
            script_path = os.path.join(os.path.dirname(__file__), 'vision_nav_coordinator.py')
            self.vision_coordinator_process = subprocess.Popen(
                ['python3', script_path, '--ros-args', '-p', f'detection_mode:={mode}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(2)  # 等待节点启动
        except Exception as e:
            self.get_logger().error(f'启动视觉检测协调节点失败: {e}')
    
    def stop_vision_coordinator(self):
        """停止视觉检测协调节点"""
        if self.vision_coordinator_process:
            self.get_logger().info('🛑 停止视觉检测协调节点')
            try:
                self.vision_coordinator_process.terminate()
                self.vision_coordinator_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.vision_coordinator_process.kill()
                self.vision_coordinator_process.wait()
            except Exception as e:
                self.get_logger().warn(f'停止视觉协调节点时出错: {e}')
            finally:
                self.vision_coordinator_process = None
    
    def switch_vision_mode(self, mode):
        """切换视觉检测模式（需要重启节点）"""
        self.stop_vision_coordinator()
        time.sleep(1)
        self.start_vision_coordinator(mode)
    
    def execute_grasp(self):
        """执行抓取流程"""
        self.get_logger().info('🤖 开始执行抓取流程...')
        self.state = 'GRASPING'
        
        try:
            # 切换到工作目录
            workspace_dir = os.path.expanduser('~/ros2_ws')
            os.chdir(workspace_dir)
            
            # 步骤1: 打开夹爪
            self.get_logger().info('步骤1: 打开夹爪')
            subprocess.run([
                'ros2', 'topic', 'pub', '-1',
                '/ax12a_arm/gripper_cmd',
                'control_msgs/msg/GripperCommand',
                '{position: 100.0, max_effort: 0.5}'
            ], check=True)
            time.sleep(1)
            
            # 步骤2: 云台控制
            self.get_logger().info('步骤2: 调整云台')
            subprocess.run([
                'ros2', 'topic', 'pub', '-1',
                '/pan_tilt_cmd_deg',
                'pan_tilt_msgs/msg/PanTiltCmdDeg',
                '{yaw: 0.0, pitch: 35.0, speed: 5}'
            ], check=True)
            time.sleep(2)
            
            # 步骤3: 机械臂运动到初始位置
            self.get_logger().info('步骤3: 机械臂运动到初始位置')
            subprocess.run([
                'ros2', 'topic', 'pub', '-1',
                '/ax12a_arm/trajectory',
                'trajectory_msgs/msg/JointTrajectory',
                '{joint_names: [\'joint1\',\'joint2\',\'joint3\',\'joint4\'], points: [{positions: [0.0, 0.0, 1.05, 0.95], time_from_start: {sec: 5.0}}]}'
            ], check=True)
            time.sleep(5)
            
            # 步骤4: 启动ArUco识别
            self.get_logger().info('步骤4: 启动ArUco识别')
            aruco_script = os.path.join(workspace_dir, 'src', 'aruco_pose_estimator.py')
            aruco_process = subprocess.Popen(
                ['python3', aruco_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            time.sleep(7)
            
            # 步骤5: 机械臂运动到抓取位置
            self.get_logger().info('步骤5: 机械臂运动到抓取位置')
            subprocess.run([
                'ros2', 'topic', 'pub', '-1',
                '/ax12a_arm/trajectory',
                'trajectory_msgs/msg/JointTrajectory',
                '{joint_names: [\'joint1\',\'joint2\',\'joint3\',\'joint4\'], points: [{positions: [0.0, -1.5, 1.05, 0.95], time_from_start: {sec: 5.0}}]}'
            ], check=True)
            time.sleep(5)
            
            # 步骤6: 启动X/Y轴PID校准
            self.get_logger().info('步骤6: 启动X/Y轴PID校准')
            x_pid_script = os.path.join(workspace_dir, 'src', 'X_PID.py')
            y_pid_script = os.path.join(workspace_dir, 'src', 'Y_PID.py')
            x_pid_process = subprocess.Popen(
                ['python3', x_pid_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 创建新的进程组
            )
            y_pid_process = subprocess.Popen(
                ['python3', y_pid_script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 创建新的进程组
            )
            time.sleep(20)  # 等待校准完成
            
            # 步骤7: 关闭夹爪（抓取）
            self.get_logger().info('步骤7: 关闭夹爪（抓取）')
            subprocess.run([
                'ros2', 'topic', 'pub', '-1',
                '/ax12a_arm/gripper_cmd',
                'control_msgs/msg/GripperCommand',
                '{position: 25.0, max_effort: 0.5}'
            ], check=True)
            time.sleep(2)
            
            # 步骤8: 恢复云台
            self.get_logger().info('步骤8: 恢复云台')
            subprocess.run([
                'ros2', 'topic', 'pub', '-1',
                '/pan_tilt_cmd_deg',
                'pan_tilt_msgs/msg/PanTiltCmdDeg',
                '{yaw: 0.0, pitch: 0.0, speed: 5}'
            ], check=True)
            time.sleep(2)
            # 清理子进程 - 优雅关闭
            self.get_logger().info('清理子进程...')
            
            # 停止ArUco进程
            try:
                if aruco_process.poll() is None:  # 进程仍在运行
                    aruco_process.terminate()
                    aruco_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                aruco_process.kill()
                aruco_process.wait()
            except Exception as e:
                self.get_logger().warn(f'关闭ArUco进程时出错: {e}')
            
            # 停止X_PID进程
            try:
                if x_pid_process.poll() is None:  # 进程仍在运行
                    try:
                        pgid = os.getpgid(x_pid_process.pid)
                        os.killpg(pgid, signal.SIGTERM)
                        x_pid_process.wait(timeout=3)
                    except ProcessLookupError:
                        # 进程组不存在，直接终止进程
                        x_pid_process.terminate()
                        x_pid_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                try:
                    pgid = os.getpgid(x_pid_process.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    x_pid_process.kill()
                x_pid_process.wait()
            except ProcessLookupError:
                pass  # 进程已经结束
            except Exception as e:
                self.get_logger().warn(f'关闭X_PID进程时出错: {e}')
            
            # 停止Y_PID进程
            try:
                if y_pid_process.poll() is None:  # 进程仍在运行
                    try:
                        pgid = os.getpgid(y_pid_process.pid)
                        os.killpg(pgid, signal.SIGTERM)
                        y_pid_process.wait(timeout=3)
                    except ProcessLookupError:
                        # 进程组不存在，直接终止进程
                        y_pid_process.terminate()
                        y_pid_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                try:
                    pgid = os.getpgid(y_pid_process.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    y_pid_process.kill()
                y_pid_process.wait()
            except ProcessLookupError:
                pass  # 进程已经结束
            except Exception as e:
                self.get_logger().warn(f'关闭Y_PID进程时出错: {e}')
            
            self.get_logger().info('子进程清理完成')
            
            self.get_logger().info('✅ 抓取流程完成')
            self.state = 'NAVIGATING'
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 抓取流程失败: {e}')
            self.state = 'NAVIGATING'
            return False
    
    def execute_place(self):
        """执行放置流程"""
        self.get_logger().info('📦 开始执行放置流程...')
        self.state = 'PLACING'
        
        try:
            # 打开夹爪（放置物品）
            self.get_logger().info('打开夹爪（放置物品）')
            subprocess.run([
                'ros2', 'topic', 'pub', '-1',
                '/ax12a_arm/gripper_cmd',
                'control_msgs/msg/GripperCommand',
                '{position: 100.0, max_effort: 0.5}'
            ], check=True)
            time.sleep(2)
            
            self.get_logger().info('✅ 放置流程完成')
            self.state = 'NAVIGATING'
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 放置流程失败: {e}')
            self.state = 'NAVIGATING'
            return False
    
    def send_waypoint(self, index):
        """发送指定索引的途径点"""
        if index >= len(self.waypoints):
            self.get_logger().info('🎉 所有点导航完成！')
            try:
                self.stop_vision_coordinator()
            except Exception as e:
                self.get_logger().warn(f'清理视觉协调节点时出错: {e}')
            time.sleep(2)
            try:
                self.destroy_node()
                rclpy.shutdown()
            except Exception as e:
                # 可能已经关闭，忽略错误
                pass
            return
        
        x, y, yaw = self.waypoints[index]
        waypoint_num = index + 1
        
        # 根据waypoint索引执行不同操作
        if index == 0:
            # 点1: 启动交通灯检测（直接模式，不需要vision_coordinator）
            self.get_logger().info(f'📍 导航到点{waypoint_num}: ({x}, {y}) [启动交通灯检测]')
            self.detection_mode = 'traffic_light'
            self.detection_state = 'RESUME'
            self.stop_count = 0
            self.go_count = 0
        elif index == 1:
            # 点2: 停止视觉检测
            self.get_logger().info(f'📍 导航到点{waypoint_num}: ({x}, {y})')
            self.detection_mode = None  # 禁用检测
        elif index == 2:
            # 点3: 正常导航，到达后执行抓取
            self.get_logger().info(f'📍 导航到点{waypoint_num}: ({x}, {y})')
        elif index == 3:
            # 点4: 导航过程中已有stop牌子检测，到达后执行放置
            self.get_logger().info(f'📍 导航到点{waypoint_num}: ({x}, {y}) [stop牌子检测中]')
        else:
            self.get_logger().info(f'📍 导航到点{waypoint_num}: ({x}, {y})')
        
        pose = self.create_pose(x, y, yaw)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.current_goal_index = index
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.handle_goal_response(f, index))
    
    def handle_goal_response(self, future, index):
        """处理目标响应"""
        goal_handle = future.result()
        waypoint_num = index + 1
        
        if not goal_handle.accepted:
            self.get_logger().error(f'❌ 点{waypoint_num}被拒绝，跳过')
            self.send_waypoint(index + 1)
            return
        
        self.get_logger().info(f'✅ 点{waypoint_num}被接受')
        self.current_goal_handle = goal_handle
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.handle_result(f, index))
    
    def handle_result(self, future, index):
        """处理结果"""
        result = future.result()
        waypoint_num = index + 1
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ 到达点{waypoint_num}')
            
            # 点3到达后执行抓取
            if index == 2:  # 点3
                self.get_logger().info('⏸️  在点3暂停，执行抓取流程...')
                if self.execute_grasp():
                    self.get_logger().info('▶️  抓取完成，启动stop牌子检测，继续导航到点4')
                    # 在导航到点4之前启动stop牌子检测（直接模式）
                    self.detection_mode = 'stop_sign'
                    self.detection_state = 'RESUME'
                    self.stop_count = 0
                    self.no_stop_count = 0
                else:
                    self.get_logger().warn('⚠️  抓取失败，但继续导航')
                # 继续导航到点4
                self.send_waypoint(index + 1)
            
            # 点4到达后执行放置
            elif index == 3:  # 点4
                self.get_logger().info('⏸️  在点4暂停，执行放置流程...')
                self.detection_mode = None  # 停止stop牌子检测
                if self.execute_place():
                    self.get_logger().info('▶️  放置完成，继续导航到点5')
                else:
                    self.get_logger().warn('⚠️  放置失败，但继续导航')
                # 继续导航到点5
                self.send_waypoint(index + 1)
            
            else:
                # 其他点直接继续
                self.send_waypoint(index + 1)
        
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'❌ 点{waypoint_num}导航失败')
            # 继续下一个点
            self.send_waypoint(index + 1)
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'⚠️  点{waypoint_num}被取消')
            # 如果是因为视觉检测暂停，不自动继续
            if not self.is_paused_by_vision:
                self.send_waypoint(index + 1)
        
        # 清除当前目标句柄
        self.current_goal_handle = None


def main():
    rclpy.init()
    node = WaypointPublisher()
    
    # 等待导航服务器
    if not node.nav_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('导航服务器未响应')
        return
    
    node.get_logger().info(f'✅ 已连接导航服务器，将导航 {len(node.waypoints)} 个点')
    
    # 开始第一个点
    node.send_waypoint(0)
    
    # 保持运行
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在关闭...')
    finally:
        # 清理
        try:
            node.stop_vision_coordinator()
        except Exception as e:
            node.get_logger().warn(f'清理视觉协调节点时出错: {e}')
        
        try:
            node.destroy_node()
        except Exception as e:
            node.get_logger().warn(f'销毁节点时出错: {e}')
        
        try:
            rclpy.shutdown()
        except Exception as e:
            # rclpy可能已经被关闭，忽略错误
            pass


if __name__ == '__main__':
    main()
