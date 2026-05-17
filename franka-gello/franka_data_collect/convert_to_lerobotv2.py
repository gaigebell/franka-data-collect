import json
import os
import pathlib
import shutil
from typing import Dict, List, Tuple

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import subprocess
import sys
from pathlib import Path
# 新增导入：用于四元数转欧拉角
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
# ======================
# 配置区
# ======================
ROBOT_TYPE = "franka_panda"
FPS = 20
IMAGE_HEIGHT, IMAGE_WIDTH = 256, 256

# 【修改】维度变更: 7 (Pose: x,y,z, roll,pitch,yaw) + 2 (Gripper) = 8
# 原逻辑: pos(3) + quat(4) + grip(2) = 9
# 新逻辑: pos(3) + euler(3) + grip(2) = 8
STATE_DIM = 8  
ACTION_DIM = 7

CHUNK_SIZE = 100

CAMERA_NAMES = ["wrist_camera"]
CAMERA_KEYS = {
    "wrist_camera": "observation.images.wrist_view",
}

# 【修改】定义 State 的字段名称
# 顺序: x, y, z, roll, pitch, yaw, gripper_1, gripper_2
# 注意：这里将原来的 ori_x/y/z/w 替换为欧拉角名称，以明确现在是 3 维朝向
STATE_NAMES = [
    "x", "y", "z", 
    "roll", "pitch", "yaw", 
    "gripper_qpos_1", "gripper_qpos_2"
]
ACTION_NAMES = [
    "x", "y", "z", 
    "roll", "pitch", "yaw", 
    "gripper"
]

def decode_and_resize_image(img: np.ndarray, target_size=(IMAGE_WIDTH, IMAGE_HEIGHT)) -> np.ndarray:
    if img is None or img.size == 0:
        return np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
    
    h, w = img.shape[:2]
    ratio = min(float(target_size[1])/h, float(target_size[0])/w)
    new_h, new_w = int(h * ratio), int(w * ratio)
    
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((target_size[1], target_size[0], 3), dtype=np.uint8)
    
    top = (target_size[1] - new_h) // 2
    left = (target_size[0] - new_w) // 2
    
    canvas[top:top+new_h, left:left+new_w] = resized
    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

def read_video_frames(video_path: pathlib.Path) -> np.ndarray:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise IOError(f"Cannot open video: {video_path}")
    
    frames = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        processed = decode_and_resize_image(frame)
        frames.append(processed)
    
    cap.release()
    assert len(frames), f'len(frames) can not be equal to zero!'
    return np.stack(frames, axis=0)

def write_video(frames: np.ndarray, output_path: pathlib.Path, fps: int = 20):
    if frames.size == 0:
        print(f"⚠️ Skipping empty video: {output_path}")
        return

    T, H, W, C = frames.shape
    assert C == 3 and frames.dtype == np.uint8, "Expected uint8 RGB frames"

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo",
        "-s", f"{W}x{H}", "-pix_fmt", "rgb24", "-r", str(fps),
        "-i", "-",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-preset", "fast", "-crf", "23",
        str(output_path)
    ]

    try:
        process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, bufsize=10**8)
        process.stdin.write(frames.tobytes())
        process.stdin.close()
        stderr = process.stderr.read()
        ret = process.wait()
        if ret != 0:
            raise RuntimeError(f"FFmpeg failed: {stderr.decode()}")
    except FileNotFoundError:
        raise EnvironmentError("FFmpeg not found. Please install it.")
    except Exception as e:
        print(f"❌ Failed to write video {output_path}: {e}")
        raise

def load_task_instruction(episode_folder: pathlib.Path) -> str:
    instr_file = episode_folder / "instruction.txt"
    if not instr_file.exists():
        print(f"⚠️ Warning: No instruction.txt found in {episode_folder}, using 'Unknown Task'")
        return "Unknown Task"
    
    with open(instr_file, "r", encoding="utf-8") as f:
        content = f.read().strip()
        if not content:
            return "Unknown Task"
        if not content.endswith("."):
            content += "."
        return content

def quaternion_to_euler_array(quat_array: np.ndarray) -> np.ndarray:
    """
    将 (N, 4) 的四元组数组 [x, y, z, w] 转换为 (N, 3) 的欧拉角数组 [roll, pitch, yaw] (弧度制)。
    使用 'xyz' 顺序 (即 roll=pitch=yaw 对应 x,y,z 轴旋转)。
    """
    # scipy 期望的顺序通常是 [x, y, z, w]
    # 如果输入已经是 [x, y, z, w]，直接传入
    rotation = R.from_quat(quat_array)
    # as_euler 返回 [roll, pitch, yaw] 默认顺序是 'xyz'
    euler_array = rotation.as_euler('xyz', degrees=False)
    return euler_array

def load_episode_data(episode_folder: pathlib.Path) -> Tuple[np.ndarray, np.ndarray]:
    """
    从 data.parquet 读取状态数据，并将四元组朝向转换为欧拉角。
    返回: (states, actions) 均为 8 维
    """
    pq_file = episode_folder / "data.parquet"
    if not pq_file.exists():
        raise FileNotFoundError(f"Missing data.parquet in {episode_folder}")
    
    table = pq.read_table(pq_file)
    df = table.to_pandas()
    
    # 原始列名 (假设采集代码存的是四元组)
    original_pos_cols = ["pos_x", "pos_y", "pos_z"]
    original_quat_cols = ["ori_x", "ori_y", "ori_z", "ori_w"]
    original_grip_cols = ["gripper_1", "gripper_2"]
    
    # 检查列是否存在
    required_cols = original_pos_cols + original_quat_cols + original_grip_cols
    try:
        # 提取位置
        pos_data = df[original_pos_cols].to_numpy(dtype=np.float64)
        # 提取四元组
        quat_data = df[original_quat_cols].to_numpy(dtype=np.float64)
        # 提取夹爪
        grip_data = df[original_grip_cols].to_numpy(dtype=np.float64)
    except KeyError as e:
        available_cols = list(df.columns)
        raise KeyError(f"Missing columns in parquet. Expected {required_cols}. Available: {available_cols}. Error: {e}")
    
    # 【核心修改】将四元组转换为欧拉角
    if len(quat_data) > 0:
        euler_data = quaternion_to_euler_array(quat_data)
    else:
        euler_data = np.empty((0, 3), dtype=np.float64)
    
    # 拼接成新的 8 维数据: [x, y, z, roll, pitch, yaw, g1, g2]
    state_data = np.concatenate([pos_data, euler_data, grip_data], axis=1)
    
    T = len(state_data)
    if T < 2:
        return np.empty((0, STATE_DIM), dtype=np.float64), np.empty((0, ACTION_DIM), dtype=np.float64)
    
    # 因果对齐：obs = t, action = t+1 (导致 t+1 的状态)
    # 注意：动作空间是 7 维表示, 与liberoplus对齐
    states = deepcopy(state_data[:-1])  # (T-1, 8) #(gripper : +0.02, +0.02)
    states[:, -1] = -states[:, -1] #与liberoplus的数据对齐

    actions = deepcopy(state_data[1:][:-1])  # (T-1, 7)
    actions[:,-1] = ((state_data[1:][-2] + state_data[1:][-1])< 0.005).astype(actions.dtype)#1=close, 0=open
    return states, actions

def save_episode_parquet(
    episode_index: int,
    states: np.ndarray,
    actions: np.ndarray,
    task_index: int,
    output_chunk_dir: pathlib.Path
):
    T = len(states)
    if T == 0:
        return

    # 构建数据字典
    data = {
        "observation.state": [s for s in states],
        "action": [a for a in actions],
        "timestamp": np.arange(T, dtype=np.float64) / FPS,
        "task_index": np.full(T, task_index, dtype=np.int64),
        "episode_index": np.full(T, episode_index, dtype=np.int64),
        "index": np.arange(T, dtype=np.int64),
        "next.done": np.concatenate([np.zeros(T - 1, dtype=bool), [True]]),
        "next.reward": np.concatenate([np.zeros(T - 1), [1.0]]),
    }

    # 【修改】Schema 维度更新为 8
    schema = pa.schema([
        ("observation.state", pa.list_(pa.float64(), STATE_DIM)),
        ("action", pa.list_(pa.float64(), ACTION_DIM)),
        ("timestamp", pa.float64()),
        ("task_index", pa.int64()),
        ("episode_index", pa.int64()),
        ("index", pa.int64()),
        ("next.done", pa.bool_()),
        ("next.reward", pa.float64()),
    ])

    table = pa.table(data, schema=schema)
    parquet_path = output_chunk_dir / f"episode_{episode_index:06d}.parquet"
    pq.write_table(table, parquet_path)

def main(input_path: str, output_path: str):
    input_root = pathlib.Path(input_path)
    output_root = pathlib.Path(output_path)
    
    if output_root.exists():
        pass # 可选清理逻辑
    
    meta_dir = output_root / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)
    
    episode_folders = sorted([d for d in input_root.iterdir() if d.is_dir()])
    
    if not episode_folders:
        print(f"❌ No episode folders found in {input_path}")
        return

    total_episodes = len(episode_folders)
    print(f" Found {total_episodes} episodes.")

    task_to_index: Dict[str, int] = {}
    episode_tasks = []

    print(" Scanning instructions...")
    for ep_folder in episode_folders:
        instr = load_task_instruction(ep_folder)
        episode_tasks.append(instr)
        if instr not in task_to_index:
            task_to_index[instr] = len(task_to_index)
    total_tasks = len(task_to_index)
    print(f"✅ Identified {total_tasks} unique tasks.")

    with open(meta_dir / "tasks.jsonl", "w", encoding="utf-8") as f:
        for task, idx in sorted(task_to_index.items(), key=lambda x: x[1]):
            f.write(json.dumps({"task_index": idx, "task": task}) + "\n")
            
    total_frames = 0
    episode_path = meta_dir / "episodes.jsonl"
    if episode_path.exists():
        episode_path.unlink()

    for ep_idx, ep_folder in enumerate(episode_folders):
        print(f" Processing episode {ep_idx}/{total_episodes}: {ep_folder.name}")
        
        try:
            # 1. 加载并转换状态数据 (9维 -> 8维)
            states, actions = load_episode_data(ep_folder)
            T = len(states)
            if T == 0:
                print(f"⚠️ Skip ep {ep_idx}: No valid state data.")
                continue
            
            total_frames += T
            task_index = task_to_index[episode_tasks[ep_idx]]
            
            # 2. 加载视频数据
            video_src = ep_folder / "rgb_stream.mp4"
            frames_main = read_video_frames(video_src)
            
            # 帧数对齐检查
            # 注意：因为 states/actions 少了一帧 (T-1)，而视频是 T 帧
            # 原代码逻辑: assert len(frames_main)-1==T (这里的 T 是 len(states))
            # 所以 frames_main 应该是 T+1 长度，取前 T 帧与 states 对齐
            if len(frames_main) != T + 1:
                print(f"⚠️ Frame mismatch: Video has {len(frames_main)}, States have {T}. Truncating/Aligning...")
                min_len = min(len(frames_main) - 1, T)
                frames_main = frames_main[:min_len]
                states = states[:min_len]
                actions = actions[:min_len]
                T = min_len

            frames_main = frames_main[:T]
            
            # 3. 确定 Chunk
            chunk_id = ep_idx // CHUNK_SIZE
            data_chunk_dir = output_root / "data" / f"chunk-{chunk_id:03d}"
            data_chunk_dir.mkdir(parents=True, exist_ok=True)
            video_base_dir = output_root / "videos" / f"chunk-{chunk_id:03d}"

            # 4. 保存视频
            cam_name = "wrist_camera"
            frames = frames_main
            video_key = CAMERA_KEYS[cam_name]
            video_dir = video_base_dir / video_key
            video_path = video_dir / f"episode_{ep_idx:06d}.mp4"
            
            write_video(frames, video_path, fps=FPS)

            # 5. 保存 Parquet (8维)
            save_episode_parquet(ep_idx, states, actions, task_index, data_chunk_dir)

            # 6. 记录 Episode 信息
            with open(episode_path, "a", encoding="utf-8") as f_ep:
                json.dump({
                    "episode_index": ep_idx,
                    "task_index": task_index,
                    "length": T
                }, f_ep)
                f_ep.write("\n")

        except Exception as e:
            print(f"❌ Error processing {ep_folder.name}: {e}")
            import traceback
            traceback.print_exc()
            continue

    # ======================
    # 生成 Meta Info
    # ======================
    total_chunks = (total_episodes + CHUNK_SIZE - 1) // CHUNK_SIZE if total_episodes > 0 else 0
    
    info = {
        "codebase_version": "v2.0",
        "robot_type": ROBOT_TYPE,
        "total_episodes": total_episodes,
        "total_frames": total_frames,
        "total_tasks": total_tasks,
        "total_videos": len(CAMERA_NAMES) * total_episodes,
        "total_chunks": total_chunks,
        "chunks_size": CHUNK_SIZE,
        "fps": float(FPS),
        "splits": {"train": f"0:{total_episodes}"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
        "features": {
            "observation.images.wrist_view": {
                "dtype": "video",
                "shape": [IMAGE_HEIGHT, IMAGE_WIDTH, 3],
                "names": ["height", "width", "channel"],
                "video_info": {
                    "video.fps": float(FPS),
                    "video.codec": "h264",
                    "video.pix_fmt": "yuv420p",
                    "video.is_depth_map": False,
                    "has_audio": False
                }
            },
            "observation.state": {
                "dtype": "float64",
                "shape": [STATE_DIM], # 8
                "names": STATE_NAMES,
            },
            "action": {
                "dtype": "float64",
                "shape": [ACTION_DIM], # 7
                "names": STATE_NAMES,
            },
            "timestamp": {"dtype": "float64", "shape": [1]},
            "task_index": {"dtype": "int64", "shape": [1]},
            "episode_index": {"dtype": "int64", "shape": [1]},
            "index": {"dtype": "int64", "shape": [1]},
            "next.reward": {"dtype": "float64", "shape": [1]},
            "next.done": {"dtype": "bool", "shape": [1]},
        }
    }
    
    with open(meta_dir / "info.json", "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)

    modality = {
        "state": {
            **{name: {"start": i, "end": i+1} for i, name in enumerate(STATE_NAMES)}
        },
        "action": {
            **{name: {"start": i, "end": i+1} for i, name in enumerate(ACTION_NAMES)}
        },
        "video": {
            "wrist_view": {"original_key": CAMERA_KEYS["wrist_camera"]},
        },
        "annotation": {
            "human.action.task_description": {"original_key": "task_index"}
        }
    }
    
    with open(meta_dir / "modality.json", "w", encoding="utf-8") as f:
        json.dump(modality, f, indent=2)

    print(f"\n✅ Conversion complete!")
    print(f"   Output dir: {output_root}")
    print(f"   Episodes: {total_episodes}, Frames: {total_frames}, Tasks: {total_tasks}")
    print(f"   State/Action Dim: {STATE_DIM} (Converted Quat -> Euler)")
    print(f"   Chunks: {total_chunks}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Convert Template A to LeRobot V2 Format (8-Dim State/Action)")
    parser.add_argument("--input_dir", required=True, help="Path to root folder containing episode subfolders")
    parser.add_argument("--output_dir", required=True, help="Output path for LeRobot v2 dataset")
    args = parser.parse_args()

    main(args.input_dir, args.output_dir)