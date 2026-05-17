import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import argparse
import os
import sys
import time
import datetime
import threading
import queue
import termios
import tty
import shutil  # 新增：用于删除文件夹

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import cv2
import imageio.v2 as imageio

# 假设 message_filters 已安装并可用
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState 

# 配置基础路径
BASE_DIR = "/media/hcp/disk/projects/hcp_panda_real_data_easy_v2"

# 定义数据结构大小 (用于预分配内存)
DTYPE_META = np.dtype([
    ('img_sec', np.int32),
    ('img_nsec', np.int32),
    ('pose_sec', np.int32),
    ('pose_nsec', np.int32),
    ('grip_sec', np.int32),
    ('grip_nsec', np.int32),
    ('diff_pose_ms', np.float64),
    ('diff_grip_ms', np.float64),
    ('g1', np.float64),
    ('g2', np.float64),
    ('x', np.float64),
    ('y', np.float64),
    ('z', np.float64),
    ('ox', np.float64),
    ('oy', np.float64),
    ('oz', np.float64),
    ('ow', np.float64)
])

class DataCollectorNode(Node):
    def __init__(self, fps=20, buffer_size=10000, task_name="default_task"):
        super().__init__('data_collector_node')
        
        self.fps = fps
        self.frame_interval = 1.0 / fps
        self.task_name = task_name
        
        # --- 状态控制 ---
        self.is_recording = False
        self.shutdown_event = threading.Event() 
        
        # --- 线程安全队列 ---
        self.data_queue = queue.Queue(maxsize=2048) 
        
        # --- 资源句柄 ---
        self.current_save_folder = None
        
        # --- 内存优化缓冲 (Meta数据) ---
        self.max_buffer_size = buffer_size
        self.meta_buffer = np.zeros(buffer_size, dtype=DTYPE_META)
        self.buffer_index = 0
        self.buffer_lock = threading.Lock()
        
        # --- 图像帧缓存列表 ---
        self.frame_buffer = [] 
        self.frame_buffer_lock = threading.Lock()

        # FPS 控制
        self.last_save_time_worker = 0.0

        # QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅者
        sub_image = message_filters.Subscriber(self, Image, '/camera/camera/color/image_rect_raw', qos_profile=qos_profile)
        sub_pose = message_filters.Subscriber(self, PoseStamped, '/franka_robot_state_broadcaster/current_pose', qos_profile=qos_profile)
        sub_gripper = message_filters.Subscriber(self, JointState, '/franka_gripper/joint_states', qos_profile=qos_profile)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [sub_image, sub_pose, sub_gripper],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        # 启动后台工作线程
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info(f"Data Collector Started (Task: {task_name}, Target FPS: {fps}, Buffer Size: {buffer_size}).")
        self.print_controls()

    def print_controls(self):
        print("\n" + "="*50)
        print(f"  TASK: {self.task_name}")
        print("  CONTROLS")
        print("="*50)
        print("  [s] : Start Recording")
        print("        (If recording: DISCARD current & Restart)")
        print("  [e] : Stop & Save")
        print("  [q] : Quit Program")
        print("="*50 + "\n")

    def sync_callback(self, img_msg, pose_msg, gripper_msg):
        if not self.is_recording:
            return

        try:
            packet = (img_msg, pose_msg, gripper_msg)
            self.data_queue.put(packet, block=False)
        except queue.Full:
            pass

    def _worker_loop(self):
        self.get_logger().info("Worker thread started.")
        
        while not self.shutdown_event.is_set():
            try:
                try:
                    packet = self.data_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                img_msg, pose_msg, gripper_msg = packet

                # A. FPS 控制
                current_time = time.time()
                if current_time - self.last_save_time_worker < self.frame_interval:
                    continue 
                self.last_save_time_worker = current_time

                # B. 解析图像数据
                img_np = None
                try:
                    h, w = img_msg.height, img_msg.width
                    expected_len = h * w * 3
                    if len(img_msg.data) < expected_len:
                        continue

                    if img_msg.encoding == 'rgb8':
                        img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(h, w, 3)
                    elif img_msg.encoding == 'bgr8':
                        img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(h, w, 3)
                        img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
                    else:
                        img_np = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(h, w, 3)
                        if img_msg.encoding.startswith('bgr'):
                            img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
                    
                except Exception:
                    continue

                # C. 解析状态并写入内存缓冲
                if self.is_recording:
                    with self.buffer_lock:
                        if self.buffer_index >= self.max_buffer_size:
                            self.get_logger().warn("Buffer full! Stopping recording automatically.")
                            self.is_recording = False
                            continue

                        p = pose_msg.pose.position
                        o = pose_msg.pose.orientation
                        pose_vec = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

                        g1, g2 = 0.0, 0.0
                        try:
                            names = list(gripper_msg.name)
                            positions = list(gripper_msg.position)
                            if 'fr3_finger_joint1' in names and 'fr3_finger_joint2' in names:
                                idx1 = names.index('fr3_finger_joint1')
                                idx2 = names.index('fr3_finger_joint2')
                                g1 = float(positions[idx1])
                                g2 = float(positions[idx2])
                            elif len(positions) >= 2:
                                g1, g2 = float(positions[0]), float(positions[1])
                        except Exception:
                            pass 

                        t_img = (img_msg.header.stamp.sec, img_msg.header.stamp.nanosec)
                        t_pose = (pose_msg.header.stamp.sec, pose_msg.header.stamp.nanosec)
                        t_grip = (gripper_msg.header.stamp.sec, gripper_msg.header.stamp.nanosec)

                        def calc_diff_ms(base, target):
                            return (target[0] - base[0]) * 1000.0 + (target[1] - base[1]) * 1e-6

                        diff_pose = calc_diff_ms(t_img, t_pose)
                        diff_grip = calc_diff_ms(t_img, t_grip)

                        idx = self.buffer_index
                        row = self.meta_buffer[idx]
                        row['img_sec'] = t_img[0]
                        row['img_nsec'] = t_img[1]
                        row['pose_sec'] = t_pose[0]
                        row['pose_nsec'] = t_pose[1]
                        row['grip_sec'] = t_grip[0]
                        row['grip_nsec'] = t_grip[1]
                        row['diff_pose_ms'] = diff_pose
                        row['diff_grip_ms'] = diff_grip
                        row['g1'] = g1
                        row['g2'] = g2
                        row['x'] = pose_vec[0]
                        row['y'] = pose_vec[1]
                        row['z'] = pose_vec[2]
                        row['ox'] = pose_vec[3]
                        row['oy'] = pose_vec[4]
                        row['oz'] = pose_vec[5]
                        row['ow'] = pose_vec[6]
                        
                        self.buffer_index += 1
                    
                    with self.frame_buffer_lock:
                        self.frame_buffer.append(img_np)

            except Exception as e:
                self.get_logger().error(f"Worker thread error: {e}")
                import traceback
                traceback.print_exc()

        self.get_logger().info("Worker thread stopping.")

    def save_data(self):
        if not self.is_recording and len(self.frame_buffer) == 0 and self.buffer_index == 0:
            return

        was_recording = self.is_recording
        self.is_recording = False
        
        start_wait = time.time()
        while not self.data_queue.empty() and (time.time() - start_wait < 1.0):
            time.sleep(0.05)
        
        frames_to_save = []
        count = 0
        
        with self.buffer_lock:
            count = self.buffer_index
            if count == 0:
                with self.frame_buffer_lock:
                    self.frame_buffer.clear()
                self.buffer_index = 0
                if not was_recording: 
                    return
                self.get_logger().warn("No meta data to save.")
                return

            valid_data = self.meta_buffer[:count].copy()

            with self.frame_buffer_lock:
                frames_to_save = self.frame_buffer[:]
                self.frame_buffer.clear()
            
            self.buffer_index = 0
        
        if len(frames_to_save) != count:
            self.get_logger().warn(f"Frame count mismatch: Meta={count}, Frames={len(frames_to_save)}. Truncating.")
            min_len = min(count, len(frames_to_save))
            valid_data = valid_data[:min_len]
            frames_to_save = frames_to_save[:min_len]
            count = min_len

        if count == 0:
            return

        try:
            if self.current_save_folder is None:
                timestamp_str = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
                task_dir = os.path.join(BASE_DIR, self.task_name)
                os.makedirs(task_dir, exist_ok=True)
                self.current_save_folder = os.path.join(task_dir, timestamp_str)
                os.makedirs(self.current_save_folder, exist_ok=True)

            video_path = os.path.join(self.current_save_folder, "rgb_stream.mp4")
            
            if len(frames_to_save) > 0:
                frames_to_save = [f.astype(np.uint8) for f in frames_to_save]
                imageio.mimwrite(video_path, frames_to_save, fps=self.fps, codec='libx264')
                self.get_logger().info(f"Video saved: {video_path} ({count} frames)")
            else:
                self.get_logger().warn("No frames found to save video.")

            schema = pa.schema([
                ('img_stamp_sec', pa.int32()),
                ('img_stamp_nsec', pa.int32()),
                ('pose_stamp_sec', pa.int32()),
                ('pose_stamp_nsec', pa.int32()),
                ('grip_stamp_sec', pa.int32()),
                ('grip_stamp_nsec', pa.int32()),
                ('sync_diff_pose_ms', pa.float64()),
                ('sync_diff_grip_ms', pa.float64()),
                ('gripper_1', pa.float64()),
                ('gripper_2', pa.float64()),
                ('pos_x', pa.float64()),
                ('pos_y', pa.float64()),
                ('pos_z', pa.float64()),
                ('ori_x', pa.float64()),
                ('ori_y', pa.float64()),
                ('ori_z', pa.float64()),
                ('ori_w', pa.float64())
            ])

            table = pa.table({
                'img_stamp_sec': pa.array(valid_data['img_sec']),
                'img_stamp_nsec': pa.array(valid_data['img_nsec']),
                'pose_stamp_sec': pa.array(valid_data['pose_sec']),
                'pose_stamp_nsec': pa.array(valid_data['pose_nsec']),
                'grip_stamp_sec': pa.array(valid_data['grip_sec']),
                'grip_stamp_nsec': pa.array(valid_data['grip_nsec']),
                'sync_diff_pose_ms': pa.array(valid_data['diff_pose_ms']),
                'sync_diff_grip_ms': pa.array(valid_data['diff_grip_ms']),
                'gripper_1': pa.array(valid_data['g1']),
                'gripper_2': pa.array(valid_data['g2']),
                'pos_x': pa.array(valid_data['x']),
                'pos_y': pa.array(valid_data['y']),
                'pos_z': pa.array(valid_data['z']),
                'ori_x': pa.array(valid_data['ox']),
                'ori_y': pa.array(valid_data['oy']),
                'ori_z': pa.array(valid_data['oz']),
                'ori_w': pa.array(valid_data['ow'])
            }, schema=schema)

            parquet_path = os.path.join(self.current_save_folder, "data.parquet")
            pq.write_table(table, parquet_path)
            self.get_logger().info(f"Parquet saved: {parquet_path} ({count} rows)")
            self.get_logger().info(f"Dataset saved to: {self.current_save_folder}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")
            import traceback
            traceback.print_exc()

def keyboard_listener(node: DataCollectorNode):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    node.print_controls()
    
    try:
        tty.setcbreak(fd)
        new_settings = termios.tcgetattr(fd)
        new_settings[3] = new_settings[3] & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)
        
        while rclpy.ok():
            key = sys.stdin.read(1)
            if not key:
                continue
            
            if key == 's':
                # 逻辑：如果已经在录制，丢弃当前数据（删除文件夹），然后重新开始
                if node.is_recording:
                    print("\n>>> [RESTART] Discarding current recording...")
                    
                    # 1. 停止录制状态，防止 worker 继续写入
                    node.is_recording = False
                    
                    # 2. 删除已创建的文件夹 (如果有)
                    folder_to_delete = node.current_save_folder
                    if folder_to_delete and os.path.exists(folder_to_delete):
                        try:
                            shutil.rmtree(folder_to_delete)
                            print(f"    -> Deleted folder: {folder_to_delete}")
                        except Exception as e:
                            print(f"    -> Error deleting folder: {e}")
                    else:
                        print("    -> No folder found to delete.")
                    
                    # 3. 清空内存缓冲和队列
                    with node.buffer_lock:
                        node.buffer_index = 0
                    with node.frame_buffer_lock:
                        node.frame_buffer.clear()
                    
                    while not node.data_queue.empty():
                        try:
                            node.data_queue.get_nowait()
                        except queue.Empty:
                            break
                    
                    print("    -> Buffers cleared.")
                
                # 4. 开始新的录制 (无论之前是否在录制)
                timestamp_str = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
                
                task_dir = os.path.join(BASE_DIR, node.task_name)
                os.makedirs(task_dir, exist_ok=True)
                save_folder = os.path.join(task_dir, timestamp_str)
                os.makedirs(save_folder, exist_ok=True)
                
                node.current_save_folder = save_folder
                node.is_recording = True
                
                # 确保 buffers 是干净的 (双重保险)
                with node.buffer_lock:
                    node.buffer_index = 0
                with node.frame_buffer_lock:
                    node.frame_buffer.clear()
                
                # 清空旧队列
                while not node.data_queue.empty():
                    try:
                        node.data_queue.get_nowait()
                    except queue.Empty:
                        break
                
                print(f"\n>>> [STARTED] Recording... (Saving to: {save_folder})")
                node.get_logger().info(f">>> STARTED Recording... Task: {node.task_name}")
                
            elif key == 'e':
                if not node.is_recording:
                    print("\n[WARN] Not recording.")
                    continue
                print("\n>>> [STOPPING] Flushing RAM to Disk...")
                node.save_data()
                node.is_recording = False
                node.current_save_folder = None
                print(">>> [READY] Press 's' to start new recording.")
                
            elif key == 'q':
                print("\n>>> Quitting...")
                rclpy.shutdown()
                break
            
    except Exception as e:
        node.get_logger().error(f"Keyboard listener error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nTerminal settings restored.")

def main(args=None):
    parser = argparse.ArgumentParser(description='Franka Data Collector with Task Name')
    parser.add_argument('--task_name', type=str, default='default_task', help='Name of the task for saving data (e.g., PickApple)')
    parsed_args, _ = parser.parse_known_args(args)
    
    rclpy.init(args=args)
    
    node = DataCollectorNode(fps=20, buffer_size=10000, task_name=parsed_args.task_name)
    
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
        if node.is_recording or len(node.frame_buffer) > 0:
            node.get_logger().info("Emergency stop detected. ")
        
        node.shutdown_event.set()
        node.worker_thread.join(timeout=2.0)
        
        node.destroy_node()
        executor.shutdown()

if __name__ == '__main__':
    main()