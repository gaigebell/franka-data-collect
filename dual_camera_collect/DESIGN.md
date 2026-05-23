# Dual Camera Data Collection System - 设计说明

## 1. 系统概述

双相机数据采集系统基于 franka-gello 项目架构，新增 Orbbec 前置相机支持，采用混合架构：
- **机械臂控制**：ROS 2 + Gello（保持原有方式）
- **相机数据获取**：Python 线程直接调用相机 SDK（不通过 ROS）
- **数据保存**：LeRobot v2.1 格式

## 2. 目录结构

```
franka-data-collect/
├── franka-gello/                      # 原有代码（不修改）
│   └── franka_data_collect/
│       ├── data_collect.py          # 单相机版（RealSense via ROS）
│       ├── start_gello.sh           # Gello 系统启动
│       └── start_data_collect.sh    # 数据采集启动
│
├── dual_camera_collect/              # 新增目录
│   ├── camera_interface.py          # 相机抽象基类
│   ├── orbbec_camera_driver.py      # Orbbec 相机驱动
│   ├── realsense_camera_driver.py   # RealSense 相机驱动
│   ├── camera_capture_thread.py     # 相机采集线程
│   ├── dual_camera_collector.py     # ROS 2 节点 + 主程序
│   ├── lerobot_writer.py            # LeRobot v2.1 写入器
│   └── launch/
│       └── start_dual_camera.sh     # 双相机系统启动脚本
│
└── tavp_data_collect/               # 已有（引用 OrbbecCamera）
    └── orbbec_camera_v6.py
```

## 3. 架构设计

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Dual Camera Collector                             │
│                    (混合架构)                                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────────────┐         ┌─────────────────────┐            │
│  │  ROS 2 Node          │         │  Camera Capture      │            │
│  │  (DualCollectorNode) │         │  Thread             │            │
│  │                      │         │                      │            │
│  │  - /current_pose     │         │  - OrbbecCamera     │            │
│  │  - /joint_states    │         │    (pyorbbecsdk)    │            │
│  │  - 数据写入          │         │  - RealSense SDK   │            │
│  │  - 键盘控制          │         │    (pyrealsense2)   │            │
│  │  - LeRobot 保存      │         │  → Queue            │            │
│  └──────────┬──────────┘         └──────────┬──────────┘            │
│             │                               │                        │
│             │          ┌────────────────────┘                        │
│             │          ▼                                             │
│             │   ┌─────────────────┐                                 │
│             └──│  Camera Queue    │◄──────────────┐                 │
│                 │  (Queue)        │               │                 │
│                 └────────┬────────┘               │                 │
│                          │                        │                 │
│                          ▼                        │                 │
│                 ┌─────────────────────┐           │                 │
│                 │  DataSynchronizer   │           │                 │
│                 │  同步 ROS + Camera  │◄──────────┘                 │
│                 │  写入 LeRobot      │                              │
│                 └─────────────────────┘                              │
└─────────────────────────────────────────────────────────────────────┘
```

## 4. 核心模块

### 4.1 CameraInterface (camera_interface.py)

抽象基类，定义所有相机驱动的统一接口：

| 方法 | 说明 |
|------|------|
| `start()` | 启动相机 |
| `stop()` | 停止相机 |
| `get_frame()` | 获取单帧 (color, depth, metadata) |
| `get_color_intrinsics()` | 获取彩色内参 |
| `get_depth_intrinsics()` | 获取深度内参 |
| `camera_name` | 相机名称属性 |

### 4.2 OrbbecCameraDriver (orbbec_camera_driver.py)

- 封装 `tavp_data_collect/orbbec_camera_v6.py` 中的 `OrbbecCamera` 类
- 直接调用 `pyorbbecsdk`，不通过 ROS
- 支持彩色图、深度图、点云（可选）

### 4.3 RealSenseCameraDriver (realsense_camera_driver.py)

- 封装 `pyrealsense2` SDK
- 直接调用 SDK，不通过 ROS
- 支持彩色图、深度图、点云（可选）

### 4.4 CameraCaptureThread (camera_capture_thread.py)

- 独立线程，直接调用各相机驱动的 `get_frame()`
- 实现 FPS 控制
- 通过 `queue.Queue` 将帧数据传递给主线程

### 4.5 LeRobotWriter (lerobot_writer.py)

LeRobot v2.1 格式写入器，支持：

| 数据类型 | 格式 | 说明 |
|----------|------|------|
| 视频 | MP4 (H.264) | 两个相机的彩色视频 |
| 图像 | PNG | 按帧保存的彩色图 |
| 深度 | NPY | 按帧保存的深度图 |
| 点云 | NPY | 按帧保存的点云 |
| 状态 | Parquet | 机器人状态和动作数据 |

### 4.6 DualCameraCollectorNode (dual_camera_collector.py)

ROS 2 节点：

- 订阅机械臂状态 topic（保持原有方式）
- 接收相机采集线程的帧数据
- 键盘控制录制
- 协调数据写入

## 5. 数据流

```
┌─────────────────────────────────────────────────────────────┐
│ CameraCaptureThread (相机采集线程)                           │
│                                                             │
│  OrbbecCameraDriver.get_frame() ──┐                         │
│                                   ├──▶ frames dict          │
│  RealSenseCameraDriver.get_frame() ─┘                        │
│                                                             │
│  frames.put((frames, timestamp)) ──▶ Camera Queue           │
└─────────────────────────────────────────────────────────────┘
                                            │
                                            ▼
┌─────────────────────────────────────────────────────────────┐
│ DualCameraCollectorNode (ROS 2 节点)                        │
│                                                             │
│  Queue.get() ◀──────────────────────────────┘                │
│                                                             │
│  ┌───────────────────────────────────────┐                  │
│  │ LeRobotWriter.write_frame()            │                  │
│  │                                       │                  │
│  │ 1. video_writers['orbbec'].append()   │                  │
│  │ 2. cv2.imwrite() / np.save()         │                  │
│  │ 3. state_data.append()                │                  │
│  └───────────────────────────────────────┘                  │
└─────────────────────────────────────────────────────────────┘
```

## 6. 命令行参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--enable_orbbec` | bool | True | 启用/禁用 Orbbec 相机 |
| `--enable_realsense` | bool | True | 启用/禁用 RealSense 相机 |
| `--orbbec_color` | bool | True | 保存 Orbbec 彩色图 |
| `--orbbec_depth` | bool | True | 保存 Orbbec 深度图 |
| `--orbbec_pcd` | bool | False | 保存 Orbbec 点云 |
| `--realsense_color` | bool | True | 保存 RealSense 彩色图 |
| `--realsense_depth` | bool | True | 保存 RealSense 深度图 |
| `--realsense_pcd` | bool | False | 保存 RealSense 点云 |
| `--save_video` | bool | True | 保存视频 MP4 |
| `--save_images` | bool | True | 保存图像/深度/点云 |
| `--fps` | int | 30 | 采集帧率 |
| `--task_name` | str | default_task | 任务名称 |
| `--base_dir` | str | /media/hcp/disk/data/tavp_dataset | 保存根目录 |

## 7. 数据保存结构

```
{BASE_DIR}/
└── {task_name}/
    └── {timestamp}/
        ├── videos/
        │   ├── observation.images.third_person.mp4   # Orbbec
        │   └── observation.images.wrist.mp4           # RealSense
        ├── frontImg/
        │   └── 000000.png ~ 999999.png
        ├── frontDep/
        │   └── 000000.npy ~ 999999.npy
        ├── frontPcd/
        │   └── 000000.npy ~ 999999.npy
        ├── wristImg/
        │   └── 000000.png ~ 999999.png
        ├── wristDep/
        │   └── 000000.npy ~ 999999.npy
        ├── wristPcd/
        │   └── 000000.npy ~ 999999.npy
        └── data.parquet
```

## 8. 启动流程

```
bash start_dual_camera.sh
│
├── 激活 conda gello-env
├── 加载 ROS 2 Humble
├── 加载 Franka ROS 2 Workspace
├── 加载 Gello 软件环境
│
├── 启动 Gello 系统节点 (后台)
│   ├── T0_State_Publisher
│   ├── T1_Arm_Controllers
│   ├── T2_Gripper_Manager
│   └── T3_Camera (RealSense)
│
├── 启动 dual_camera_collector.py
│   ├── 初始化 OrbbecCameraDriver
│   ├── 初始化 RealSenseCameraDriver
│   ├── 启动 CameraCaptureThread
│   └── 进入键盘控制循环
│
└── 按 q 退出时
    ├── 停止 CameraCaptureThread
    ├── 停止所有相机
    ├── 关闭写入器
    └── 杀死 Gello 后台进程
```

## 9. 复用已有代码

| 已有代码 | 复用方式 |
|----------|----------|
| `tavp_data_collect/orbbec_camera_v6.py` | 直接 import `OrbbecCamera` |
| `franka_data_collect/data_collect.py` | 参考 ROS topic 订阅和状态解析 |
| `franka_data_collect/convert_to_lerobotv2.py` | 参考四元数转欧拉角 (`scipy.spatial.transform.Rotation`) |

## 10. 待实现功能

1. **RealSense 点云生成** - 需在 `realsense_camera_driver.py` 中实现点云生成逻辑
2. **Orbbec 外参支持** - 可在 `OrbbecCameraDriver.__init__` 传入 `extrinsics` 参数
3. **多相机同步精确时间戳对齐** - 当前使用近似时间同步