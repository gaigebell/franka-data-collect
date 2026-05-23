# -*- coding: utf-8 -*-
"""
LeRobot v2.1 格式写入器
数据保存支持视频、图像、深度图、点云和 parquet 状态数据
"""

import os
import time
import json
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import imageio.v2 as imageio
import cv2
from typing import Dict

# 保存配置（运行时通过 update_config 设置）
SAVE_CONFIG = {
    'orbbec_color': True, 'orbbec_depth': True, 'orbbec_pcd': False,
    'realsense_color': True, 'realsense_depth': True, 'realsense_pcd': False,
    'save_video': True, 'save_images': True,
    'fps': 30
}


def update_config(config: dict):
    """更新保存配置"""
    SAVE_CONFIG.update(config)


class LeRobotWriter:
    """LeRobot v2.1 格式写入器"""

    def __init__(self, episode_dir: str, fps: int = 30):
        """
        初始化写入器

        Args:
            episode_dir: episode 保存目录
            fps: 帧率
        """
        self.episode_dir = episode_dir
        self.fps = fps
        self.frame_index = 0
        self.episode_index = 0

        # 创建目录结构
        os.makedirs(episode_dir, exist_ok=True)
        os.makedirs(os.path.join(episode_dir, 'videos'), exist_ok=True)
        os.makedirs(os.path.join(episode_dir, 'frontImg'), exist_ok=True)
        os.makedirs(os.path.join(episode_dir, 'frontDep'), exist_ok=True)
        os.makedirs(os.path.join(episode_dir, 'frontPcd'), exist_ok=True)
        os.makedirs(os.path.join(episode_dir, 'wristImg'), exist_ok=True)
        os.makedirs(os.path.join(episode_dir, 'wristDep'), exist_ok=True)
        os.makedirs(os.path.join(episode_dir, 'wristPcd'), exist_ok=True)

        # 视频写入器
        self.video_writers = {}
        self._init_video_writers()

        # Parquet 数据缓冲
        self.state_data = []
        self.gello_joints_data = []
        self.action_data = []
        self.meta_data = []

    def _init_video_writers(self):
        """初始化视频写入器"""
        if SAVE_CONFIG.get('save_video', True):
            if SAVE_CONFIG.get('orbbec_color', True):
                path = os.path.join(self.episode_dir, 'videos',
                                   'observation.images.third_person.mp4')
                self.video_writers['orbbec'] = imageio.get_writer(
                    path, fps=self.fps, codec='libx264'
                )
            if SAVE_CONFIG.get('realsense_color', True):
                path = os.path.join(self.episode_dir, 'videos',
                                   'observation.images.wrist.mp4')
                self.video_writers['realsense'] = imageio.get_writer(
                    path, fps=self.fps, codec='libx264'
                )

    def write_frame(self, frames: Dict[str, dict], robot_state: dict):
        """
        写入一帧数据

        Args:
            frames: {
                'orbbec': {'color': np.ndarray, 'depth': np.ndarray, 'metadata': dict},
                'realsense': {'color': np.ndarray, 'depth': np.ndarray, 'metadata': dict}
            }
            robot_state: {
                'position': [x, y, z],
                'quaternion': [x, y, z, w],
                'gripper': float,  # 二值化: 0=张开, 1=关闭
                'gello_joints': [j1, j2, j3, j4, j5, j6, j7]  # 7个关节角度
            }
        """
        idx = self.frame_index

        # 1. 保存视频帧
        if SAVE_CONFIG.get('save_video', True):
            for cam_name, writer in self.video_writers.items():
                if cam_name in frames and frames[cam_name]:
                    color = frames[cam_name].get('color')
                    if color is not None:
                        writer.append_data(color)

        # 2. 保存图像/深度/点云
        if SAVE_CONFIG.get('save_images', True):
            if 'orbbec' in frames and frames['orbbec']:
                f = frames['orbbec']
                if SAVE_CONFIG.get('orbbec_color', True) and f.get('color') is not None:
                    cv2.imwrite(
                        os.path.join(self.episode_dir, 'frontImg', f'{idx:06d}.png'),
                        cv2.cvtColor(f['color'], cv2.COLOR_RGB2BGR)
                    )
                if SAVE_CONFIG.get('orbbec_depth', True) and f.get('depth') is not None:
                    np.save(os.path.join(self.episode_dir, 'frontDep', f'{idx:06d}.npy'), f['depth'])
                if SAVE_CONFIG.get('orbbec_pcd', False) and f.get('metadata', {}).get('pointcloud') is not None:
                    np.save(os.path.join(self.episode_dir, 'frontPcd', f'{idx:06d}.npy'),
                           f['metadata']['pointcloud'])

            if 'realsense' in frames and frames['realsense']:
                f = frames['realsense']
                if SAVE_CONFIG.get('realsense_color', True) and f.get('color') is not None:
                    cv2.imwrite(
                        os.path.join(self.episode_dir, 'wristImg', f'{idx:06d}.png'),
                        cv2.cvtColor(f['color'], cv2.COLOR_RGB2BGR)
                    )
                if SAVE_CONFIG.get('realsense_depth', True) and f.get('depth') is not None:
                    np.save(os.path.join(self.episode_dir, 'wristDep', f'{idx:06d}.npy'), f['depth'])
                if SAVE_CONFIG.get('realsense_pcd', False) and f.get('metadata', {}).get('pointcloud') is not None:
                    np.save(os.path.join(self.episode_dir, 'wristPcd', f'{idx:06d}.npy'),
                           f['metadata']['pointcloud'])

        # 3. 保存状态数据
        self._save_state_action(robot_state)
        self.frame_index += 1

    def _save_state_action(self, robot_state: dict):
        """保存状态和动作数据 - 包含三种机器人数据"""
        from scipy.spatial.transform import Rotation as R

        pos = robot_state['position']  # [x, y, z]
        quat = robot_state['quaternion']  # [x, y, z, w]
        gripper = robot_state['gripper']  # 二值: 0=张开, 1=关闭
        gripper_width = robot_state.get('gripper_width', 0.0)  # 夹爪间距（米）
        gello_joints = robot_state.get('gello_joints', [0.0] * 7)  # 7个关节角度

        # 四元数转欧拉角
        r = R.from_quat(quat)
        euler = r.as_euler('xyz')

        # observation.state: [x, y, z, roll, pitch, yaw, qx, qy, qz, qw, gripper] - 11个元素
        observation_state = np.array([
            pos[0], pos[1], pos[2],
            euler[0], euler[1], euler[2],
            quat[0], quat[1], quat[2], quat[3],
            gripper
        ], dtype=np.float64)

        # observation.gello_joints: 7个关节角度
        observation_gello_joints = np.array(gello_joints, dtype=np.float64)

        # action 与 observation.state 相同（11个元素）
        action = observation_state.copy()

        self.state_data.append(observation_state)
        self.gello_joints_data.append(observation_gello_joints)
        self.action_data.append(action)
        self.meta_data.append({
            'timestamp': time.time(),
            'frame_index': self.frame_index,
            'episode_index': self.episode_index,
            'index': self.frame_index,
            'gripper_width': gripper_width
        })

    def save_parquet(self):
        """保存 parquet 文件"""
        schema = pa.schema([
            ('observation.state', pa.list_(pa.float64())),
            ('observation.gello_joints', pa.list_(pa.float64())),
            ('action', pa.list_(pa.float64())),
            ('timestamp', pa.float64()),
            ('frame_index', pa.int64()),
            ('episode_index', pa.int64()),
            ('index', pa.int64()),
            ('gripper_width', pa.float64()),
        ])

        table = pa.table({
            'observation.state': pa.array(self.state_data),
            'observation.gello_joints': pa.array(self.gello_joints_data),
            'action': pa.array(self.action_data),
            'timestamp': pa.array([m['timestamp'] for m in self.meta_data]),
            'frame_index': pa.array([m['frame_index'] for m in self.meta_data], type=pa.int64()),
            'episode_index': pa.array([m['episode_index'] for m in self.meta_data], type=pa.int64()),
            'index': pa.array([m['index'] for m in self.meta_data], type=pa.int64()),
            'gripper_width': pa.array([m.get('gripper_width', 0.0) for m in self.meta_data], type=pa.float64()),
        })

        pq.write_table(table, os.path.join(self.episode_dir, 'data.parquet'))

    def save_json(self):
        """保存 json 文件（方便检查数据）"""
        json_data = []
        for i in range(len(self.state_data)):
            state = self.state_data[i]
            gello_joints = self.gello_joints_data[i]
            meta = self.meta_data[i]
            json_data.append({
                'frame_index': int(meta['frame_index']),
                'episode_index': int(meta['episode_index']),
                'index': int(meta['index']),
                'timestamp': float(meta['timestamp']),
                'observation.state': {
                    'position': {
                        'x': float(state[0]),
                        'y': float(state[1]),
                        'z': float(state[2])
                    },
                    'euler': {
                        'roll': float(state[3]),
                        'pitch': float(state[4]),
                        'yaw': float(state[5])
                    },
                    'quaternion': {
                        'x': float(state[6]),
                        'y': float(state[7]),
                        'z': float(state[8]),
                        'w': float(state[9])
                    },
                    'gripper': float(state[10])
                },
                'observation.gello_joints': {
                    'fr3_joint1': float(gello_joints[0]),
                    'fr3_joint2': float(gello_joints[1]),
                    'fr3_joint3': float(gello_joints[2]),
                    'fr3_joint4': float(gello_joints[3]),
                    'fr3_joint5': float(gello_joints[4]),
                    'fr3_joint6': float(gello_joints[5]),
                    'fr3_joint7': float(gello_joints[6]),
                },
                'action': {
                    'position': {
                        'x': float(state[0]),
                        'y': float(state[1]),
                        'z': float(state[2])
                    },
                    'euler': {
                        'roll': float(state[3]),
                        'pitch': float(state[4]),
                        'yaw': float(state[5])
                    },
                    'quaternion': {
                        'x': float(state[6]),
                        'y': float(state[7]),
                        'z': float(state[8]),
                        'w': float(state[9])
                    },
                    'gripper': float(state[10])
                },
                'gripper_width': float(meta.get('gripper_width', 0.0))
            })

        json_path = os.path.join(self.episode_dir, 'data.json')
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(json_data, f, indent=2, ensure_ascii=False)

    def close(self):
        """关闭所有写入器并保存最终数据"""
        for writer in self.video_writers.values():
            writer.close()
        self.save_parquet()
        self.save_json()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()