# 双相机数据采集系统 - 使用说明

## 1. 快速开始

### 方式一：一键启动（推荐）

```bash
cd /home/hcp/workspace/tavp_proj/tavp_collect_data_gello/franka-data-collect/dual_camera_collect/launch
bash start_dual_camera.sh
```

这将自动：
1. 激活 conda 环境
2. 加载 ROS 2 环境
3. 启动 Gello 系统节点
4. 启动双相机数据采集程序

### 方式二：分步启动

```bash
# 1. 先启动 Gello 系统节点（终端 1）
cd /home/hcp/workspace/tavp_proj/tavp_collect_data_gello/franka-data-collect/franka-gello/franka_data_collect
bash start_gello.sh

# 2. 再启动采集程序（终端 2）
cd /home/hcp/workspace/tavp_proj/tavp_collect_data_gello/franka-data-collect/dual_camera_collect
source /opt/ros/humble/setup.bash
source ~/workspace/franka_ros2_ws/install/setup.bash
python dual_camera_collector.py --enable_orbbec true --enable_realsense true
```

## 2. 键盘控制

| 按键 | 功能 |
|------|------|
| `s` | 开始录制（创建新 episode） |
| `e` | 停止录制（保存数据） |
| `q` | 退出程序 |

### 录制流程

```
1. 运行程序 → 显示控制提示
2. 按 [s] → 开始录制（屏幕显示 "STARTED"）
3. 操作机械臂采集数据
4. 按 [e] → 停止录制（屏幕显示 "STOPPED"）
5. 按 [q] → 退出程序
```

## 3. 命令行参数

### 相机开关

| 参数 | 说明 |
|------|------|
| `--enable_orbbec true/false` | 启用/禁用 Orbbec 相机 |
| `--enable_realsense true/false` | 启用/禁用 RealSense 相机 |

### 数据类型选择

| 参数 | 说明 |
|------|------|
| `--orbbec_color true/false` | Orbbec 彩色图 |
| `--orbbec_depth true/false` | Orbbec 深度图 |
| `--orbbec_pcd true/false` | Orbbec 点云 |
| `--realsense_color true/false` | RealSense 彩色图 |
| `--realsense_depth true/false` | RealSense 深度图 |
| `--realsense_pcd true/false` | RealSense 点云 |

### 保存选项

| 参数 | 说明 |
|------|------|
| `--save_video true/false` | 保存 MP4 视频 |
| `--save_images true/false` | 保存 PNG/NPY 图像 |

### 其他配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--fps` | 30 | 采集帧率 |
| `--task_name` | default_task | 任务名称 |
| `--base_dir` | /media/hcp/disk/data/tavp_dataset | 保存根目录 |

## 4. 使用示例

### 仅使用 Orbbec 相机

```bash
python dual_camera_collector.py \
    --enable_orbbec true \
    --enable_realsense false \
    --orbbec_color true \
    --orbbec_depth true \
    --task_name orbbec_only
```

### 仅使用 RealSense 相机

```bash
python dual_camera_collector.py \
    --enable_orbbec false \
    --enable_realsense true \
    --realsense_color true \
    --realsense_depth true \
    --task_name realsense_only
```

### 双相机 + 彩色图保存

```bash
python dual_camera_collector.py \
    --enable_orbbec true \
    --enable_realsense true \
    --orbbec_color true \
    --orbbec_depth false \
    --realsense_color true \
    --realsense_depth false \
    --save_video true \
    --save_images true \
    --task_name color_only
```

### 双相机 + 所有数据

```bash
python dual_camera_collector.py \
    --enable_orbbec true \
    --enable_realsense true \
    --orbbec_color true \
    --orbbec_depth true \
    --orbbec_pcd true \
    --realsense_color true \
    --realsense_depth true \
    --realsense_pcd true \
    --save_video true \
    --save_images true \
    --task_name full_data
```

## 5. 数据输出

### 目录结构

```
{BASE_DIR}/
└── {task_name}/
    └── {timestamp}/
        ├── videos/
        │   ├── observation.images.third_person.mp4   # Orbbec 彩色视频
        │   └── observation.images.wrist.mp4           # RealSense 彩色视频
        ├── frontImg/
        │   ├── 000000.png
        │   ├── 000001.png
        │   └── ...
        ├── frontDep/
        │   ├── 000000.npy
        │   ├── 000001.npy
        │   └── ...
        ├── frontPcd/        (仅当 --orbbec_pcd true)
        │   ├── 000000.npy
        │   └── ...
        ├── wristImg/
        │   ├── 000000.png
        │   └── ...
        ├── wristDep/
        │   ├── 000000.npy
        │   └── ...
        ├── wristPcd/        (仅当 --realsense_pcd true)
        │   ├── 000000.npy
        │   └── ...
        └── data.parquet     # 机器人状态数据
```

### Parquet 数据格式

| 列名 | 类型 | 说明 |
|------|------|------|
| observation.state | list[float64] | [x, y, z, roll, pitch, yaw, gripper] |
| action | list[float64] | 与 state 相同 |
| timestamp | float64 | 时间戳 |
| frame_index | int64 | 帧索引 |
| episode_index | int64 | episode 索引 |
| index | int64 | 全局索引 |

## 6. 修改启动参数

编辑 `launch/start_dual_camera.sh` 文件中的 python 命令参数：

```bash
python dual_camera_collector.py \
    --enable_orbbec true \
    --enable_realsense true \
    --orbbec_color true \
    --orbbec_depth false \       # 修改这里
    --realsense_color true \
    --realsense_depth false \     # 修改这里
    --save_video true \
    --save_images true \
    --fps 30 \
    --task_name my_task \        # 修改这里
    --base_dir /path/to/data    # 修改这里
```

## 7. 依赖环境

- conda 环境: `gello-env`
- ROS 2 Humble
- Franka ROS 2 Workspace
- Gello 软件环境
- Python 包:
  - `rclpy`, `sensor_msgs` (ROS 2)
  - `pyorbbecsdk` (Orbbec)
  - `pyrealsense2` (RealSense)
  - `numpy`, `opencv-python`
  - `pyarrow`, `imageio`
  - `scipy` (四元数转欧拉角)

## 8. 故障排除

### 问题：Orbbec 相机无法初始化

- 检查 USB 连接
- 检查 pyorbbecsdk 是否正确安装
- 外参文件路径检查

### 问题：RealSense 相机无法初始化

- 检查 USB 连接
- 检查 pyrealsense2 是否正确安装

### 问题：ROS topic 订阅不到数据

- 确保 Gello 系统节点已启动
- 检查 `/franka_robot_state_broadcaster/current_pose` 和 `/franka_gripper/joint_states` topic 是否存在

### 问题：数据保存失败

- 检查保存目录是否有写入权限
- 检查磁盘空间是否充足