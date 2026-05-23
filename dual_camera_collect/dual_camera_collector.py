#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual Camera Data Collector

混合架构：
- 机械臂控制：ROS 2 + Gello（保持原有方式）
- 相机数据：Python 线程直接获取
- 数据保存：LeRobot v2.1 格式
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import argparse
import os
import sys
import threading
import datetime
import termios
import tty
import shutil
import queue
import copy
import logging

# 调试日志文件
DEBUG_LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'debug.log')
_debug_handler = None

def _get_debug_handler():
    global _debug_handler
    if _debug_handler is None:
        _debug_handler = logging.FileHandler(DEBUG_LOG_FILE)
        _debug_handler.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(message)s'))
    return _debug_handler

# 替换 print，在打印到终端的同时写入文件
_original_print = print
def print(*args, **kwargs):
    _original_print(*args, **kwargs)
    if DEBUG_MODE:
        msg = ' '.join(str(a) for a in args)
        logger = logging.getLogger('debug')
        logger.addHandler(_get_debug_handler())
        logger.setLevel(logging.DEBUG)
        logger.debug(msg)

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# 导入本地模块
from orbbec_camera_driver import OrbbecCameraDriver
from realsense_camera_driver import RealSenseCameraDriver
from camera_capture_thread import CameraCaptureThread
from lerobot_writer import LeRobotWriter, update_config


# ============================================================================
# 写入线程：将数据写入磁盘的操作完全独立出来，不阻塞 ROS 回调
# ============================================================================
class WriterThread(threading.Thread):
    """独立写入线程，处理所有磁盘 I/O 操作"""

    def __init__(self, write_queue: queue.Queue, writer: LeRobotWriter, debug_mode: bool = True):
        super().__init__(daemon=True)
        self.write_queue = write_queue
        self.writer = writer
        self.debug_mode = debug_mode
        self.running = True
        self._stopping = False  # 排空模式标志

    def run(self):
        while self.running:
            try:
                frames, robot_state = self.write_queue.get(timeout=0.05)
                self.writer.write_frame(frames, robot_state)
            except queue.Empty:
                # 队列为空时，如果是排空模式则退出，否则继续等待
                if self._stopping:
                    break
                continue
            except Exception as e:
                print(f"[WriterThread] 写入错误: {e}")

        # 排空模式：确保所有数据写入后再退出
        self.running = False

    def stop(self):
        self._stopping = True  # 进入排空模式，等待队列清空后自然退出


# ============================================================================
# 全局配置参数
# ============================================================================
# 调试模式：True 时输出调试信息到终端
DEBUG_MODE = True

# 夹爪二值化阈值 (mm)
GRIPPER_CLOSE_THRESHOLD_MM = 10.0
CONFIG = {
    'enable_orbbec': True,
    'enable_realsense': True,
    'orbbec_color': True,
    'orbbec_depth': True,
    'orbbec_pcd': False,
    'realsense_color': True,
    'realsense_depth': True,
    'realsense_pcd': False,
    'save_video': True,
    'save_images': True,
    'fps': 30,
    'task_name': 'default_task',
    'base_dir': '/media/hcp/disk/data/tavp_dataset',
}


class DualCameraCollectorNode(Node):
    """
    ROS 2 节点 - 双相机数据采集器
    保持原有机械臂状态订阅，新增相机采集线程
    """

    def __init__(self, config: dict):
        super().__init__('dual_camera_collector')
        self.config = config

        # 状态
        self.is_recording = False
        self.current_pose = None
        self.current_gripper = None
        self.shutdown_event = threading.Event()

        # 相机帧队列
        self.camera_queue = queue.Queue(maxsize=100)

        # 初始化相机驱动
        self.cameras = {}
        if config['enable_orbbec']:
            self.cameras['orbbec'] = OrbbecCameraDriver()
            self.get_logger().info("Orbbec 相机已初始化")
        if config['enable_realsense']:
            self.cameras['realsense'] = RealSenseCameraDriver()
            self.get_logger().info("RealSense 相机已初始化")

        if not self.cameras:
            raise RuntimeError("至少需要启用一个相机")

        # 启动相机采集线程（初始为暂停状态，录制时启动）
        self.camera_thread = CameraCaptureThread(
            self.cameras,
            self.camera_queue,
            fps=config['fps']
        )
        self.camera_thread.start()
        self.camera_thread.pause()  # 初始暂停
        self.get_logger().info(f"相机采集线程已启动 (FPS: {config['fps']}, 暂停中)")

        # ROS 订阅 - 改为直接订阅（不同步，更可靠）
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅 gello 机械臂关节状态
        self.sub_gello_joints = self.create_subscription(
            JointState, 'gello/joint_states', self._on_gello_joints, qos_profile
        )

        # 订阅末端位姿
        self.sub_pose = self.create_subscription(
            PoseStamped, '/franka_robot_state_broadcaster/current_pose', self._on_pose, qos_profile
        )

        # 订阅夹爪状态
        self.sub_gripper = self.create_subscription(
            JointState, '/franka_gripper/joint_states', self._on_gripper, qos_profile
        )

        # 订阅夹爪命令意图（操作员想要的夹爪开度，0.0=闭合，1.0=张开）
        self.sub_gripper_command = self.create_subscription(
            Float32, 'gripper/gripper_client/target_gripper_width_percent', self._on_gripper_command, qos_profile
        )

        # 同步后的最新状态
        self.current_gello_joints = None
        self.current_pose = None
        self.current_gripper = None
        self.current_gripper_command = None  # 操作员的夹爪命令意图（0.0=闭合，1.0=张开）

        # 用于保护 ROS 回调和定时器之间的并发访问
        self._state_lock = threading.Lock()

        # 数据处理定时器
        self.timer = self.create_timer(1.0 / config['fps'], self._process_data)

        # 写入器
        self.writer = None
        self.episode_dir = None

        # 写入队列（供 WriterThread 消费）
        self.write_queue = queue.Queue(maxsize=100)
        self.writer_thread = None

        self.get_logger().info("DualCameraCollector 初始化完成")

    def _on_gello_joints(self, msg: JointState):
        with self._state_lock:
            self.current_gello_joints = copy.deepcopy(msg)
        if not self.is_recording:
            if DEBUG_MODE:
                print(f"[GELLO CALLBACK] positions={list(msg.position)[:3]}")

    def _on_pose_fallback(self, msg: PoseStamped):
        """备用 pose 回调 - 当同步器不触发时使用"""
        with self._state_lock:
            self.current_pose = copy.deepcopy(msg)
        if self.is_recording:
            if DEBUG_MODE:
                p = copy.deepcopy(msg.pose.position)
                print(f"[DEBUG fallback] x={p.x:.4f} y={p.y:.4f} z={p.z:.4f}")

    def _on_pose(self, msg: PoseStamped):
        import time
        x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ros_ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        sys_ts = time.time()
        with self._state_lock:
            self.current_pose = copy.deepcopy(msg)
        if DEBUG_MODE:
            print(f"[POSE CALLBACK] x={x:.4f} y={y:.4f} z={z:.4f} recording={self.is_recording} age={sys_ts-ros_ts:.3f}s")

    def _on_gripper(self, msg: JointState):
        with self._state_lock:
            self.current_gripper = copy.deepcopy(msg)
        if not self.is_recording:
            if DEBUG_MODE:
                print(f"[GRIPPER CALLBACK] names={list(msg.name)[:3]}")

    def _on_gripper_command(self, msg: Float32):
        """订阅操作员的夹爪命令意图（0.0=闭合，1.0=张开）"""
        with self._state_lock:
            self.current_gripper_command = msg.data  # 0.0~1.0
        if not self.is_recording:
            if DEBUG_MODE:
                print(f"[GRIPPER_COMMAND] target_width_percent={msg.data:.3f}")

    def _process_data(self):
        """处理相机帧和机械臂状态，异步写入（不阻塞 ROS 回调）"""
        if not self.is_recording or self.writer is None:
            return

        try:
            frames, _timestamp = self.camera_queue.get_nowait()
        except queue.Empty:
            return

        robot_state = self._get_robot_state()
        try:
            self.write_queue.put_nowait((frames, robot_state))
        except queue.Full:
            print(f"[PROCESS_DATA] 写入队列已满，跳过帧")

    def _get_robot_state(self) -> dict:
        """获取当前机器人状态 - 包含三种数据（快速拷贝，不阻塞）"""
        with self._state_lock:
            current_pose = self.current_pose
            current_gripper = self.current_gripper
            current_gello_joints = self.current_gello_joints
            current_gripper_command = self.current_gripper_command  # 操作员的夹爪命令意图
            # 在锁内直接读取数值，避免 ROS 回调在读取过程中修改引用导致数据错乱
            pose_position = (current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z) if current_pose else (0.0, 0.0, 0.0)
            pose_orientation = (current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w) if current_pose else (0.0, 0.0, 0.0, 1.0)
            gripper_positions = list(current_gripper.position) if current_gripper else []
            # gello_joints 过滤，只保留 fr3_joint1-7
            gello_joints = [0.0] * 7
            if current_gello_joints:
                try:
                    names = list(current_gello_joints.name)
                    positions = list(current_gello_joints.position)
                    for i, name in enumerate(names):
                        if name.startswith('fr3_joint'):
                            joint_num = int(name.replace('fr3_joint', ''))
                            if 1 <= joint_num <= 7:
                                gello_joints[joint_num - 1] = positions[i]
                except Exception:
                    pass

        # 1. 末端位姿
        position = list(pose_position)
        quaternion = list(pose_orientation)

        # 2. 夹爪二值 action（使用操作员命令意图）和间距（实际物理位置）
        # 操作员摇杆 < 0.1 = 夹紧意图（action=1），否则=张开（action=0）
        gripper_action = 1.0 if (current_gripper_command is not None and current_gripper_command < 0.1) else 0.0
        gripper_width = 0.0  # 两指间距（米）
        if gripper_positions:
            try:
                if len(gripper_positions) >= 2:
                    gripper_width = gripper_positions[0] + gripper_positions[1]
            except Exception:
                pass

        return {
            'position': position,
            'quaternion': quaternion,
            'gripper': gripper_action,
            'gripper_width': gripper_width,
            'gello_joints': gello_joints
        }

    def _get_next_episode_number(self) -> str:
        """获取下一个episode编号"""
        task_dir = os.path.join(self.config['base_dir'], self.config['task_name'])
        if not os.path.exists(task_dir):
            return "data_001"

        existing = [d for d in os.listdir(task_dir) if d.startswith('data_') and d[5:].isdigit()]
        if not existing:
            return "data_001"

        max_num = max(int(d.split('_')[1]) for d in existing)
        return f"data_{max_num + 1:03d}"

    def start_recording(self):
        """开始录制"""
        if self.is_recording:
            # 丢弃当前录制
            self.is_recording = False
            if self.writer:
                self.writer.close()
            if self.episode_dir and os.path.exists(self.episode_dir):
                shutil.rmtree(self.episode_dir)

        # 重置状态变量，确保录制开始时使用新的同步数据
        # 注意：不再重置 current_pose，避免 ROS 回调来不及更新导致数据卡死
        # self.current_gello_joints = None
        # self.current_pose = None
        # self.current_gripper = None

        # 清空队列缓存，避免残留旧帧
        while not self.camera_queue.empty():
            try:
                self.camera_queue.get_nowait()
            except queue.Empty:
                break

        episode_name = self._get_next_episode_number()
        self.episode_dir = os.path.join(self.config['base_dir'], self.config['task_name'], episode_name)
        os.makedirs(self.episode_dir, exist_ok=True)

        self.writer = LeRobotWriter(self.episode_dir, fps=self.config['fps'])
        self.is_recording = True
        self.camera_thread.resume()

        # 清空写入队列（丢弃旧数据）
        while not self.write_queue.empty():
            try:
                self.write_queue.get_nowait()
            except queue.Empty:
                break

        # 启动写入线程
        self.writer_thread = WriterThread(self.write_queue, self.writer, debug_mode=DEBUG_MODE)
        self.writer_thread.start()

        self.get_logger().info(f"录制开始: {self.episode_dir}")

        # 等待一下，确保同步回调收到新数据后再开始
        import time
        time.sleep(0.5)

    def stop_recording(self):
        """停止录制"""
        if not self.is_recording:
            return
        self.is_recording = False
        self.camera_thread.pause()

        # 停止写入线程
        if self.writer_thread:
            self.writer_thread.stop()
            self.writer_thread.join(timeout=5.0)
            self.writer_thread = None

        if self.writer:
            self.writer.close()
            self.writer = None
        self.get_logger().info("录制停止")

    def shutdown(self):
        """关闭清理"""
        self.camera_thread.stop()
        for cam in self.cameras.values():
            cam.stop()
        if self.writer:
            self.writer.close()


def parse_args():
    parser = argparse.ArgumentParser(description='Dual Camera Data Collector')

    # 相机开关
    parser.add_argument('--enable_orbbec', type=lambda x: x.lower() == 'true', default=True)
    parser.add_argument('--enable_realsense', type=lambda x: x.lower() == 'true', default=True)

    # Orbbec 数据类型
    parser.add_argument('--orbbec_color', type=lambda x: x.lower() == 'true', default=True)
    parser.add_argument('--orbbec_depth', type=lambda x: x.lower() == 'true', default=True)
    parser.add_argument('--orbbec_pcd', type=lambda x: x.lower() == 'true', default=False)

    # RealSense 数据类型
    parser.add_argument('--realsense_color', type=lambda x: x.lower() == 'true', default=True)
    parser.add_argument('--realsense_depth', type=lambda x: x.lower() == 'true', default=True)
    parser.add_argument('--realsense_pcd', type=lambda x: x.lower() == 'true', default=False)

    # 保存选项
    parser.add_argument('--save_video', type=lambda x: x.lower() == 'true', default=True)
    parser.add_argument('--save_images', type=lambda x: x.lower() == 'true', default=True)

    # 其他配置
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--task_name', type=str, default='default_task')
    parser.add_argument('--base_dir', type=str, default='/media/hcp/disk/data/tavp_dataset_extra')
    parser.add_argument('--debug', action='store_true', default=False, help='启用调试打印')

    return parser.parse_args()


def print_controls():
    print("\n" + "="*50)
    print("  Dual Camera Collector")
    print("="*50)
    print("  [s] : 开始录制")
    print("  [e] : 停止录制")
    print("  [q] : 退出程序")
    print("="*50 + "\n")


def keyboard_listener(node: DualCameraCollectorNode):
    """键盘监听线程"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)

        while rclpy.ok():
            key = sys.stdin.read(1)

            if key == 's':
                node.start_recording()
            elif key == 'e':
                node.stop_recording()
            elif key == 'q':
                print("\n>>> 退出...")
                node.shutdown()
                rclpy.shutdown()
                break

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    parsed_args = parse_args()

    # 设置全局调试模式
    global DEBUG_MODE
    DEBUG_MODE = parsed_args.debug

    # 更新全局配置
    CONFIG.update({
        'enable_orbbec': parsed_args.enable_orbbec,
        'enable_realsense': parsed_args.enable_realsense,
        'orbbec_color': parsed_args.orbbec_color,
        'orbbec_depth': parsed_args.orbbec_depth,
        'orbbec_pcd': parsed_args.orbbec_pcd,
        'realsense_color': parsed_args.realsense_color,
        'realsense_depth': parsed_args.realsense_depth,
        'realsense_pcd': parsed_args.realsense_pcd,
        'save_video': parsed_args.save_video,
        'save_images': parsed_args.save_images,
        'fps': parsed_args.fps,
        'task_name': parsed_args.task_name,
        'base_dir': parsed_args.base_dir,
    })

    # 更新写入器配置
    update_config({
        'orbbec_color': CONFIG['orbbec_color'],
        'orbbec_depth': CONFIG['orbbec_depth'],
        'orbbec_pcd': CONFIG['orbbec_pcd'],
        'realsense_color': CONFIG['realsense_color'],
        'realsense_depth': CONFIG['realsense_depth'],
        'realsense_pcd': CONFIG['realsense_pcd'],
        'save_video': CONFIG['save_video'],
        'save_images': CONFIG['save_images'],
        'fps': CONFIG['fps'],
    })

    rclpy.init(args=args)

    try:
        node = DualCameraCollectorNode(CONFIG)
        print_controls()

        executor = MultiThreadedExecutor()
        executor.add_node(node)

        listener_thread = threading.Thread(target=keyboard_listener, args=(node,))
        listener_thread.daemon = True
        listener_thread.start()

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node.shutdown()
            executor.shutdown()

    except Exception as e:
        print(f"错误: {e}")
        rclpy.shutdown()


if __name__ == '__main__':
    main()