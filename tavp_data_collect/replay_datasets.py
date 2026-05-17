# -*- coding: utf-8 -*-
import franky
from franky import Robot, Gripper, JointMotion, CartesianMotion, Affine
import numpy as np
import time
import os
import json
import sys
from typing import List, Dict, Any

# (导入恢复脚本，假设它在同一目录下)
try:
    from recover import recover_robot
except ImportError:
    print("警告: 未找到 'recover.py'。如果机械臂出错，请手动恢复。")
    # 定义一个占位函数，以便在未找到时代码也能运行
    def recover_robot(ip):
        print(f"尝试恢复机器人 {ip} (占位函数)...")
        try:
            robot = Robot(ip)
            robot.recover_from_errors()
            print("机器人已恢复。")
        except Exception as e:
            print(f"自动恢复失败: {e}")


# --- 1. 配置参数 ---
ROBOT_IP = "172.16.0.2"

# (!!重要!!) 
# 请将此路径修改为您使用“录制代码”生成的数据集 episode 目录
# 例如: "/media/hcp/disk/data/tavp_dataset/put_item_in_drawer/episode_30"
EPISODE_TO_REPLAY = "/media/hcp/disk/data/tavp_dataset/pick_and_place_1118/episode_0"
# "/media/hcp/disk/data/tavp_dataset/collect_fruits/episode_0" 


# --- 2. 初始化 ---
def initialize_robot(ip):
    """初始化 Robot 和 Gripper 对象。"""
    print(f"正在连接 Franka 机械臂 (IP: {ip})...")
    robot = Robot(ip)
    robot.recover_from_errors()
    gripper = Gripper(ip)
    print("Franka 机械臂连接成功。")
    return robot, gripper

# --- 3. 加载录制的动作数据 ---
def load_recorded_actions(episode_dir: str) -> List[Dict[str, Any]]:
    """
    从指定的 episode 目录中加载所有录制的 action (.json) 文件。
    """
    print(f"正在从 {episode_dir} 加载录制的动作...")
    observation_dir = os.path.join(episode_dir, "observation")
    if not os.path.exists(observation_dir):
        print(f"错误: 找不到 observation 目录: {observation_dir}", file=sys.stderr)
        sys.exit(1)

    actions = []
    timestep = 0
    while True:
        action_file = os.path.join(observation_dir, f"{timestep}.json")
        if not os.path.exists(action_file):
            # 停止加载，因为找不到下一个时间步的文件
            # (这标志着轨迹的结束)
            break
        
        try:
            with open(action_file, 'r') as f:
                action_data = json.load(f)
                # 检查关键数据是否存在
                if "joint_position" not in action_data or "gripper_grasped" not in action_data:
                     print(f"警告: {action_file} 缺少 'joint_position' 或 'gripper_grasped'，已跳过。")
                     continue
                actions.append(action_data)
            timestep += 1
        except Exception as e:
            print(f"加载 {action_file} 时出错: {e}")
            break
            
    if not actions:
        print(f"错误: 在 {observation_dir} 中没有找到任何有效的 action (.json) 文件。", file=sys.stderr)
        sys.exit(1)
        
    print(f"成功加载 {len(actions)} 个动作 (Timesteps 0 到 {len(actions) - 1})。")
    return actions

# --- 4. 回放轨迹 ---
def replay_recorded_trajectory(robot: Robot, gripper: Gripper, actions: List[Dict[str, Any]]):
    """
    根据加载的动作列表，控制机械臂和夹爪复现轨迹。
    """
    print("\n--- 开始回放录制的轨迹 ---")
    
    # 根据第一个时间步的记录，设置夹爪的初始状态
    try:
        initial_grasped_state = bool(actions[0]['gripper_grasped'])
        is_grasped = initial_grasped_state
        print(f"设置初始夹爪状态: {'闭合' if is_grasped else '张开'}")
        
        if is_grasped:
             # 如果初始是抓取状态，我们执行抓取动作
             print("执行: 初始抓取 (Grasp)")
             gripper.grasp(width=0.0, speed=0.02, force=20.0, epsilon_outer=1.0)
        else:
             # 如果初始是张开状态，我们执行张开动作
             print("执行: 初始张开 (Open)")
             gripper.open(speed=0.02)
        time.sleep(0.5) # 等待夹爪动作完成
             
    except KeyError:
        print("错误: 第一个动作文件中缺少 'gripper_grasped' 键。程序中止。", file=sys.stderr)
        return
    except Exception as e:
        print(f"设置初始夹爪状态时出错: {e}")
        print("请检查机械臂和夹爪连接。")
        return

    # 遍历所有加载的动作（时间步）
    # 录制代码在移动 *之前* 采样，所以我们直接使用当前时间步的动作
    for i, action_data in enumerate(actions):
        
        print(f"\n--- 正在执行时间步 {i}/{len(actions) - 1} ---")
        
        # 1. 关节移动
        try:
            # target_q = np.array(action_data["joint_position"])
            # print(f"移动关节到: {np.round(target_q, 4)}")
            t = np.array(action_data["cartesian_xyz"])
            q = np.array(action_data["cartesian_quat_xyzw"])
            # q = [np.array(q[3]), np.array(q[0]), np.array(q[1]), np.array(q[2])]  # 转为 wxyz 格式
            motion = CartesianMotion(
                    Affine(t, q),  # quat_wxyz
                    relative_dynamics_factor=0.1)
            # 使用与录制时相似的动力学参数
            # motion = JointMotion(target=target_q, relative_dynamics_factor=0.07) 
            robot.move(motion)
            time.sleep(0.3) # 模拟录制时的暂停

        except KeyError:
            print(f"警告: 时间步 {i} 缺少 'joint_position' 数据，跳过移动。")
            continue
        except Exception as e:
            print(f"关节移动失败: {e}")
            print("尝试从错误中恢复...")
            robot.recover_from_errors()
            print("恢复成功，等待1秒后继续...")
            time.sleep(1)
            
        # 2. 夹爪控制
        try:
            # 获取当前时间步记录的 *目标* 夹爪状态
            target_gripper_grasped = bool(action_data['gripper_grasped'])
            
            # 比较当前实际状态 (is_grasped) 和 目标状态 (target_gripper_grasped)
            if is_grasped == False and target_gripper_grasped == True:
                # 状态从 张开 -> 闭合
                print("执行: 夹爪闭合 (Grasp)")
                gripper.grasp(width=0.0, speed=0.02, force=20.0, epsilon_outer=1.0)
                time.sleep(0.3) # 模拟录制时的暂停
                is_grasped = True # 更新当前实际状态
            
            elif is_grasped == True and target_gripper_grasped == False:
                # 状态从 闭合 -> 张开
                print("执行: 夹爪张开 (Open/Release)")
                gripper.open(speed=0.02)
                time.sleep(0.3) # 模拟录制时的暂停
                is_grasped = False # 更新当前实际状态
            else:
                # 状态未改变，无需动作
                print(f"夹爪状态保持: {'闭合' if is_grasped else '张开'}")
                
        except KeyError:
             print(f"警告: 时间步 {i} 缺少 'gripper_grasped' 数据，跳过夹爪动作。")
             
    print("\n[✓] 轨迹回放完成！")


# --- 5. 主函数 ---
def main_replay():
    """主回放函数"""
    if not EPISODE_TO_REPLAY or not os.path.exists(EPISODE_TO_REPLAY):
        print(f"错误: 请确保 'EPISODE_TO_REPLAY' 变量已正确设置。", file=sys.stderr)
        print(f"当前设置的无效路径: {EPISODE_TO_REPLAY}")
        sys.exit(1)

    # 1. 加载录制的动作
    actions = load_recorded_actions(EPISODE_TO_REPLAY)

    # 2. 初始化机器人
    robot, gripper = initialize_robot(ROBOT_IP)
    
    try:
        print("\n回放将在 3 秒后开始...")
        print("请确保机械臂周围环境安全！")
        time.sleep(3.0)
        
        # 3. 执行回放
        replay_recorded_trajectory(robot, gripper, actions)

    except Exception as e:
        print(f"\n回放过程中发生严重错误: {e}", file=sys.stderr)
    
    finally:
        # 4. 清理
        robot.stop()
        print("机械臂已停止。")
        print("回放程序执行完毕。")

if __name__ == "__main__":
    # 确保机械臂处于良好状态
    print(f"正在恢复机器人 {ROBOT_IP}...")
    recover_robot(ROBOT_IP)
        
    print(f"\n准备回放轨迹: {EPISODE_TO_REPLAY}")
    main_replay()
