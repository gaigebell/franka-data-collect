# script_1_record_trajectory.py
# ---------------------------------
# 阶段 1: 手动示教轨迹录制 (已完善)
# ---------------------------------

from franky import *
import time
import pickle
import json  # [修改] 导入 json
import threading
import numpy as np
import sys
import recover
from replay_keypoint_orbbec_v3 import main as replay_main

# --- 1. 配置参数 ---
# 机械臂的IP地址 (请确保与您的FCI设置一致)
ROBOT_IP = "172.16.0.2"

# 轨迹保存的文件名
TRAJECTORY_FILE = None

# --- 4. 主函数: 录制轨迹 ---
# 记录一个pos
def record_trajectory(robot):

    trajectory_data = []
    timestep = 0

    print("按 's' 保存当前位置，按 'q' 退出保存")
    while True:
        print(f"timestep:{timestep}")
        key = input("输入指令：").strip().lower()

    
        if key == 's':
            # 1. 读取机器人和夹爪的当前状态
            robot_state = robot.state
            gripper_state = gripper.state

            # 2. 封装成 'action' 字典
            cartesian_pose_matrix = robot_state.O_T_EE

            # 提取 XYZ (单位：米)
            cartesian_xyz = cartesian_pose_matrix.translation

            # 提取 四元数 (x, y, z, w)
            cartesian_quat_xyzw = np.array(cartesian_pose_matrix.quaternion)

            # 夹爪宽度 (单位：米)
            gripper_width = gripper_state.width

            # 夹爪开合状态 (0=闭合, 1=张开)
            gripper_grasped = input("夹爪状态 (1=grasp): ").strip()
            if gripper_grasped not in ['0', '1']:
                print("无效输入，默认设置为 0 (open)")
                gripper_grasped = 0
            
            # [修改] JSON 不支持 numpy 数组，必须先转换为 list
            action_data = {
                # 7轴关节角度 (rad) -> list
                "joint_position": np.array(robot_state.q).tolist(),

                # 笛卡尔坐标 (m) -> list
                "cartesian_xyz": np.array(cartesian_xyz).tolist(),

                # 四元数 (x, y, z, w) -> list
                "cartesian_quat_xyzw": cartesian_quat_xyzw.tolist(),

                # 夹爪实际宽度 (m)
                "gripper_position": float(gripper_width), # 确保是 float

                # 夹爪开合状态 (0/1)
                "gripper_grasped": int(gripper_grasped) # 确保是 int 或 str，视需求而定
            }

            # 3. 存入轨迹列表
            trajectory_data.append(action_data)

            # 实时显示录制状态
            print(f"已录制帧数: {len(trajectory_data)} | 时间步: {timestep} | 关节角度 (rad): {np.round(robot_state.q, 4)} | 夹爪:{gripper_grasped}|夹爪宽度 (m): {gripper_width:.4f} ", end="")
            timestep += 1
        elif key == 'q':
            break
        else:
            print("无效输入，请按 's' 保存或 'q' 退出")

    if trajectory_data:
        # [修改] 使用 json 保存
        # ensure_ascii=False 防止中文乱码（如果有），indent=4 用于美化输出格式
        with open(TRAJECTORY_FILE, 'w', encoding='utf-8') as f:
            json.dump(trajectory_data, f, ensure_ascii=False, indent=4)
            
        print(f"[✓] 轨迹录制完成！")
        print(f"    共 {len(trajectory_data)} 帧数据已成功保存到: {TRAJECTORY_FILE}")

if __name__ == "__main__":
    print(f"正在尝试连接 Franka 机械臂 (IP: {ROBOT_IP})...")

    robot = Robot(ROBOT_IP)
    robot.recover_from_errors()
    gripper = Gripper(ROBOT_IP)

    print("正在进行夹爪 Homing (回零)...")
    # gripper.homing()

    # 尝试读取一次，确保夹爪连接正常
    print(f"夹爪连接成功，当前宽度: {gripper.width:.4f} 米")
    print("Franka 机械臂连接成功。")

    # recover.recover_robot(ROBOT_IP)

    # print(f"正在录制第 {i+1} 次轨迹...")
    for i in range(0,1):
        # i = 30
        # [修改] 后缀名为 .json
        TRAJECTORY_FILE = f"/home/hcp/tavp_proj/tavp_collect_data/trajectory/unscrew_cap/{i}.json"
        record_trajectory(robot)