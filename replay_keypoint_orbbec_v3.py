# -*- coding: utf-8 -*-
import franky
from franky import Robot, Gripper, JointMotion
from orbbec_camera_v4 import OrbbecCamera 
import cv2
import numpy as np
import pickle
import time
import os
import sys
import json
from typing import List, Dict, Any
from recover import recover_robot


# --- 1. 配置参数 ---
ROBOT_IP = "172.16.0.2"
DATASET_BASE_DIR = "/media/hcp/disk/data/tavp_dataset/unscrew_cap/"     
# press_buttons"   pick_and_place_dataset
TRAJECTORY_FILE = None
EXTRINSICS_FILE = "/home/hcp/tavp_proj/T_base_orbbec_1119.npy" 
is_grasped = False 


# --- 2. 初始化
def initialize_robot(ip):
    print(f"正在连接 Franka 机械臂 (IP: {ip})...")
    robot = Robot(ip)
    robot.recover_from_errors()
    gripper = Gripper(ip)
    print("Franka 机械臂连接成功。")
    return robot, gripper


def get_next_episode_dir(base_dir):
    os.makedirs(base_dir, exist_ok=True)
    i = 0
    while True:
        episode_dir = os.path.join(base_dir, f"episode_{i}")
        if not os.path.exists(episode_dir):
            os.makedirs(episode_dir)
            os.makedirs(os.path.join(episode_dir, "frontImg"), exist_ok=True)
            os.makedirs(os.path.join(episode_dir, "frontDeep"), exist_ok=True)
            os.makedirs(os.path.join(episode_dir, "observation"), exist_ok=True)
            os.makedirs(os.path.join(episode_dir, "frontPcd"), exist_ok=True) 
            return episode_dir
        i += 1

def get_robot_action(robot, gripper):
    global is_grasped
    robot_state = robot.state
    gripper_state = gripper.state
    cartesian_pose_matrix = robot_state.O_T_EE
    cartesian_xyz = cartesian_pose_matrix.translation
# TODO: verify quaternion order  xyzw
    quat_xyzw = np.array(cartesian_pose_matrix.quaternion)
    # cartesian_quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
    gripper_width = gripper_state.width
    gripper_grasped = 1 if is_grasped else 0
    return {
        "joint_position": robot_state.q,
        "cartesian_xyz": cartesian_xyz,
        "cartesian_quat_xyzw": quat_xyzw, #cartesian_quat_xyzw,
        "gripper_position": gripper_width,
        "gripper_grasped": gripper_grasped
    }


# --- 3. 快照函数 (关键修改) ---
def take_snapshot(robot: Robot, gripper: Gripper, 
                  camera: OrbbecCamera, # (修改) 传入相机对象
                  episode_dir: str, timestep: int):
    
    # 1. 获取 Action
    action_data = get_robot_action(robot, gripper)


    try:
        # 2. (修改) 从相机类获取帧
        # camera.get_frames() 已经包含了同步和对齐逻辑
        color_frame, depth_frame = camera.get_frames()
        
        # 3. (修改) 从帧解码图像
        # color_image_to_save (RGB, uint8), depth_image_to_save (uint16, mm)
        color_image_to_save, depth_image_to_save = camera.get_images_from_frames(color_frame, depth_frame)

    except RuntimeError as e:
        print(f"\n!!!!!!!!!!!!!!!!!!!!!!!!!!警告: Timestep {timestep} 获取帧失败: {e}")
        raise # 强制抛出异常，停止执行

    # 打印分辨率信息 (可选)
    color_width = color_frame.get_width()
    color_height = color_frame.get_height()
    depth_width = depth_frame.get_width()
    depth_height = depth_frame.get_height()
    print(f"Color 分辨率: {color_width} x {color_height}", end=" | ")
    print(f"Aligned Depth 分辨率: {depth_width} x {depth_height}", end=" | ")


    # 4. 保存 图像 和 Action 路径
    color_path = os.path.join(episode_dir, "frontImg", f"{timestep}.png")
    depth_path = os.path.join(episode_dir, "frontDeep", f"{timestep}.npy")
    pcd_path = os.path.join(episode_dir, "frontPcd", f"{timestep}.npy") # (新增) 点云路径
    action_path = os.path.join(episode_dir, "observation", f"{timestep}.json")

    try:
        # (修改) 将RGB转为BGR进行cv2保存
        color_image_bgr_final = cv2.cvtColor(color_image_to_save, cv2.COLOR_RGB2BGR) 
        cv2.imwrite(color_path, color_image_bgr_final)
        
        # 保存深度图 (uint16, mm)
        np.save(depth_path, depth_image_to_save)
        
        # (修改) 保存 Action (与原始脚本一致)
        action_data_json = {}
        for key, value in action_data.items():
            if isinstance(value, np.ndarray):
                action_data_json[key] = value.tolist()
            else:
                action_data_json[key] = value
        with open(action_path, 'w') as f:
            json.dump(action_data_json, f, indent=4)
        
        # ---------------------------------
        # 5. (修改) 点云生成与保存
        # ---------------------------------
        # 调用类方法, 从对齐的深度图 (mm) 生成点云
        # 并直接转换为世界坐标系 (m)
        pcd_world = camera.get_aligned_point_cloud(
            depth_image_to_save, 
            to_world_frame=True
        )
        
        # 保存世界坐标系点云 (float32, m)
        np.save(pcd_path, pcd_world)
        # ---------------------------------

    except Exception as e:
        print(f"\n错误: Timestep {timestep} 写入(Img/Deep/Pcd/Json)文件失败: {e}")
         
    print(f"  [录制中 - Timestep {timestep}] | 采集中... | PCD Shape: {pcd_world.shape}", end='\r')


def load_trajectory_pkl(file_path: str) -> List[Dict[str, Any]]:
    print(f"正在加载轨迹文件: {file_path}...")
    with open(file_path, 'rb') as f:
        trajectory = pickle.load(f)
    
    if not isinstance(trajectory, list) or not all(isinstance(p, dict) and "joint_position" in p for p in trajectory):
        print("错误: 轨迹文件格式不正确，应为包含 'joint_position' 字典的列表。", file=sys.stderr)
        sys.exit(1)
            
    print(f"成功加载 {len(trajectory)} 个轨迹点。")
    return trajectory

def load_trajectory(file_path: str) -> List[Dict[str, Any]]:
    print(f"正在加载轨迹文件: {file_path}...")
    
    # [修改] 改为读取 json 文件 ('r' 模式, utf-8)
    with open(file_path, 'r', encoding='utf-8') as f:
        trajectory = json.load(f)
    
    if not isinstance(trajectory, list) or not all(isinstance(p, dict) and "joint_position" in p for p in trajectory):
        print("错误: 轨迹文件格式不正确，应为包含 'joint_position' 字典的列表。", file=sys.stderr)
        sys.exit(1)
            
    print(f"成功加载 {len(trajectory)} 个轨迹点。")
    return trajectory

def replay_trajectory(robot: Robot, gripper: Gripper, trajectory: List[Dict[str, Any]], 
                      camera: OrbbecCamera, # (修改) 传入相机对象
                      cam_extrinsics: np.ndarray, 
                      episode_dir: str):
    print("\n--- 开始关节轨迹复现 (主线程) ---")
    global is_grasped
    is_grasped = False
    
    for i, waypoint in enumerate(trajectory):
        # 动作执行前: 触发数据采集
        print(f"\n主线程 点 {i+1}/{len(trajectory)}: 触发数据采集 (Timestep: {i})...")
        # (修改) 传入 camera 对象
        take_snapshot(robot, gripper, camera, episode_dir, i)
        
        # 1. 执行关节移动
        target_q = np.array(waypoint["joint_position"])
        print(f"主线程 点 {i+1}/{len(trajectory)}: 移动关节到 {np.round(target_q, 4)} rad...")
        
        motion = JointMotion(target=target_q, relative_dynamics_factor=0.07) 
        robot.move(motion)
        time.sleep(0.3) 
        
        # 2. 执行夹爪动作 
        current_gripper_grasped = int(waypoint['gripper_grasped']) 
        if is_grasped==False and current_gripper_grasped != 0:
            gripper.grasp(width=0, speed=0.02, force=20.0, epsilon_outer=10.0)  # epsilon_outer=1.0
            print(f"主线程 点 {i+1}/{len(trajectory)}: 夹爪闭合完成。")
            time.sleep(0.3)
            is_grasped = True
        elif  is_grasped==True and current_gripper_grasped == 0:
            gripper.open(speed=0.02)
            print(f"主线程 release 点 {i+1}/{len(trajectory)}: 夹爪张开完成。")
            time.sleep(0.3)
            is_grasped = False

    # (修改) 采集最后一个点的快照 
    print(f"\n主线程 点 {len(trajectory)}: 触发数据采集 (Timestep: {len(trajectory)})...")
    take_snapshot(robot, gripper, camera, episode_dir, len(trajectory)) # 结束时的快照


# --- 5. 主函数 ---

def main():
      

    # 1. 加载轨迹
    trajectory = load_trajectory(TRAJECTORY_FILE)

    # 2. 初始化
    episode_dir = get_next_episode_dir(DATASET_BASE_DIR)
    robot, gripper = initialize_robot(ROBOT_IP)
    camera = None # 先声明
    
    try:
        # (修改) 加载相机外参
        cam_extrinsics = np.load(EXTRINSICS_FILE)

        camera = OrbbecCamera(
            color_width=1280, 
            color_height=720, 
            color_fps=30,
            depth_width=640, 
            depth_height=576, 
            depth_fps=30,
            enable_alignment=True,
            extrinsics=cam_extrinsics  # 在此处传入外参
        )
        
        print("初始化完毕... 1秒后开始复现...")
        time.sleep(3.0) 

        # 5. 主线程: 执行机械臂动作
        replay_trajectory(robot, gripper, trajectory, camera, # (修改) 传入 camera
                          cam_extrinsics, 
                          episode_dir)

        print(f"\n[✓] 轨迹录制完成！")
        # (修改) 修正了路点计数
        print(f"    共 {len(trajectory) + 1} 个时间步 (timestep 0 到 {len(trajectory)}) 数据已保存到: {episode_dir}")

    finally:
        # 9. 清理
        if camera:
            camera.stop() # (修改) 调用类的 stop 方法
        else:
            print("相机未成功初始化。")
        
        robot.stop()
        print("机械臂已停止。")
        print("程序执行完毕。")

if __name__ == "__main__":
    for i in range(5, 11):
    # i = 30
        TRAJECTORY_FILE = f"/home/hcp/tavp_proj/tavp_collect_data/trajectory/unscrew_cap/{i}.json"  # press_buttons
        recover_robot(ROBOT_IP)
        print(f"正在复现第 {i} 号轨迹 ({TRAJECTORY_FILE})...")
        # print("5 秒后开始...")
        # time.sleep(5)
        main()
        # recover_robot(ROBOT_IP)
