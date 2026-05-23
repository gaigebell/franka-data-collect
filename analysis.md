# Franka 真机数据收集项目分析

本仓库包含两个独立的 Franka 真实机器人数据采集项目：

- `franka-gello`：基于 ROS2 与 Gello 控制环境的实时数据录制
- `tavp_data_collect`：基于 `franky` 机械臂控制与 Orbbec 相机的轨迹示教和重放数据采集

---

## 一、总体对比

### 共同目标

- 采集 Franka 机械臂真机数据
- 关联视觉信息与机器人状态
- 为后续训练、分析、数据集构建提供原始数据
- 强调现场实际采集，依赖真实传感器与机器人执行

### 关键差异

| 维度 | franka-gello | tavp_data_collect |
|---|---|---|
| 运行环境 | ROS2 + Gello + Franka ROS2 | franky Python 控制 + Orbbec 相机 |
| 数据采集方式 | 在线话题同步录制 | 轨迹示教 + 重放触发采集 |
| 存储格式 | MP4 + Parquet | PNG/NPY/JSON + NPY 点云 |
| 数据目标 | 传感器流与状态同步分析 | 任务数据集与轨迹回放 |
| 核心逻辑 | `data_collect.py` | `record_trajectory_keypoint.py` / `replay_keypoint_orbbec_v3.py` |

---

## 二、`franka-gello` 项目深度拆解

### 1. 项目结构

- `franka-gello/franka_data_collect/`
  - `data_collect.py`：数据采集程序主入口
  - `load_data.py`：Parquet 数据加载与同步差分析示例
  - `convert_to_lerobotv2.py`：数据格式转换脚本
  - `start_gello.sh`：启动 ROS2 与 Gello/Franka 节点
  - `start_data_collect.sh`：启动数据采集脚本
  - `setup.sh`：环境准备脚本
  - `view_d405.py`：简单 RealSense 话题查看器
  - `readme.md`：基本使用说明

### 2. 运行与启动流程

- `start_gello.sh`：
  - 激活 `gello-env` conda 环境
  - source ROS2 Humble 工作区
  - 启动 Gello 相关 ROS2 launch 节点：
    - `franka_gello_state_publisher`
    - `franka_fr3_arm_controllers`
    - `franka_gripper_manager`
    - `realsense2_camera`
- `start_data_collect.sh`：
  - 激活 `datacollect` conda 环境
  - 加载 ROS2 与 Franka workspace
  - 启动 `data_collect.py` 指定任务名

### 3. 核心类与数据采集逻辑

#### `DataCollectorNode`

- 继承 `rclpy.node.Node`
- 订阅三个 ROS2 话题：
  - 彩色图像 `/camera/camera/color/image_rect_raw`
  - 机器人位姿 `/franka_robot_state_broadcaster/current_pose`
  - 夹爪状态 `/franka_gripper/joint_states`
- 使用 `message_filters.ApproximateTimeSynchronizer` 做近似时间同步
- 通过 `queue.Queue` 缓存同步数据包
- 启动后台工作线程 `_worker_loop` 处理队列并保存帧与状态

#### 采集与记录控制

- `s`：开始录制；若正在录制则重启并丢弃当前数据
- `e`：停止录制并写入磁盘
- `q`：退出程序

#### 数据缓冲设计

- 内存预分配 `meta_buffer`：固定长度 `buffer_size=10000`
- `frame_buffer`：保存 RGB 图像数组
- `buffer_lock` / `frame_buffer_lock`：确保线程安全
- `data_queue`：接收同步回调数据包，防止回调阻塞

#### 时间同步与帧率控制

- 使用 `ApproximateTimeSynchronizer` 将图像、位姿、夹爪近似对齐，允许 `slop=0.1s`
- 在 `worker_loop` 中按固定 `fps=20` 做帧率控制
- 保存时还记录同步偏差字段：
  - `sync_diff_pose_ms`
  - `sync_diff_grip_ms`

#### 数据记录输出

- 视频：`rgb_stream.mp4`
- Parquet：`data.parquet`
  - 包含时间戳、同步差、夹爪值、位置、姿态四元数
- 输出目录结构：`BASE_DIR/<task_name>/<timestamp>/`

### 4. 数据格式与后处理

#### Parquet schema

- 时间戳字段：`img_stamp_*`, `pose_stamp_*`, `grip_stamp_*`
- 同步差：`sync_diff_pose_ms`, `sync_diff_grip_ms`
- 机器人状态：`pos_x/y/z`, `ori_x/y/z/w`
- 夹爪：`gripper_1`, `gripper_2`

#### 后处理脚本

- `load_data.py`：用于读取 Parquet 并检查同步差分布
- `convert_to_lerobotv2.py`：
  - 读取 `data.parquet`
  - 将四元数转换为欧拉角
  - 生成下一个时间步的 `state` / `action`
  - 适配目标训练格式（`observation.state`, `action`, `next.done`, `next.reward`）

### 5. 设计优点与风险点

#### 优点

- 直接基于 ROS2 topic 采集传感器流，适合真实在线数据采集
- 有键盘控制录制，使得人工示教时可灵活触发
- 通过 Parquet 存储结构化元数据，便于后续分析与训练
- `load_data.py` 提供同步质量检验示例

#### 风险点与局限

- 同步可靠性依赖 `ApproximateTimeSynchronizer`，若话题延迟或丢帧，仍可能出现时间错配
- 图像与状态并未绑定到同一个序列号，仅靠时间戳近似对齐
- 数据写入为视频 + Parquet，若需像素级逐帧随机访问不够便利
- 代码没有明确的异常恢复流程，ROS topic / 相机异常可能导致采集中断
- `BASE_DIR` 写死路径，可移植性较差

---

## 三、`tavp_data_collect` 项目深度拆解

### 1. 项目结构

- `orbbec_camera_v4.py`：Orbbec 相机基础封装
- `orbbec_camera_v6.py`：4K/深度流与对齐增强版本
- `record_trajectory_keypoint.py`：手动示教轨迹录制
- `replay_keypoint_orbbec_v3.py`：轨迹重放 + 视觉数据采集
- `replay_datasets.py`：用于回放已录制 dataset 的脚本
- `recover.py`：机器人恢复安全姿态辅助脚本
- `README.md`：项目说明

### 2. 任务流程

#### 1) 轨迹采集

- 通过 `record_trajectory_keypoint.py` 手动记录动作序列
- 录制内容包括：
  - 机器人关节位置 `joint_position`
  - 末端笛卡尔位置 `cartesian_xyz`
  - 末端方向四元数 `cartesian_quat_xyzw`
  - 夹爪开合状态 `gripper_grasped`
  - 夹爪宽度 `gripper_position`
- 录制方式为人工按键 `s` 保存当前状态，`q` 结束
- 输出 JSON 轨迹文件

#### 2) 轨迹重放采集

- 通过 `replay_keypoint_orbbec_v3.py` 重放轨迹
- 先初始化机器人与 Orbbec 相机
- 每一步先采集视觉数据，再执行轨迹动作
- 采集数据包括：
  - RGB 图像 `frontImg/<timestep>.png`
  - 深度图 `frontDeep/<timestep>.npy`
  - 点云 `frontPcd/<timestep>.npy`
  - 状态/动作 JSON `observation/<timestep>.json`
- 采用外参 `T_base_camera` 进行点云生成与世界坐标转换

### 3. Orbbec 相机模块实现

#### `orbbec_camera_v4.py`

- 使用 `pyorbbecsdk`
- 创建彩色与深度流
- 加载相机内参与畸变参数
- 支持对齐（Depth-to-Color）
- 提供 RGB / Depth frame 解析接口

#### `orbbec_camera_v6.py`

- 在 v4 基础上增强：
  - 支持 4K 彩色 MJPG 流
  - 支持可选深度流 `enable_depth`
  - 适配 `enable_alignment` 与 `enable_depth` 的交互
  - 增加帧获取重试与超时逻辑
  - 更健壮的硬件内参获取

### 4. 数据结构与格式

#### 轨迹文件

- JSON 格式，包含每个时间步的机器人状态、笛卡尔位置、四元数、夹爪状态
- 这类数据用于后续重放/动作耦合训练

#### 视觉数据与点云

- RGB 图像：PNG，BGR -> RGB 转换清晰
- 深度图：`np.save()` 的 `.npy`，保留原始深度值
- 点云：通过深度与相机内参/外参生成，保存为 `.npy`
- 目录结构明确，适合分段存储多个 episode

#### 数据集回放脚本

- `replay_datasets.py`：用于回放录制结果，检查动作与观察是否一致
- `view_d405.py`：ROS2 话题可视化工具，辅助检查 RealSense 话题在 `franka-gello` 项目中的图像流

### 5. 设计优点与风险点

#### 优点

- 提供完整“示教 -> 重放 -> 视觉采集”流水线
- 数据组织为目录式 episode，适合任务数据集构建与可视化检查
- 有点云生成逻辑，可用于空间理解与 3D 数据训练
- `orbbec_camera_v6.py` 体现了对硬件兼容性的增强与 4K 优化

#### 风险点与局限

- `replay_keypoint_orbbec_v3.py` 先采集再移动，导致采集中状态与动作可能存在偏差
- 动作重放时使用 `CartesianMotion` + `Affine`，但轨迹文件仍保留关节角度和笛卡尔位姿，存在格式不一致隐患
- JSON 数据记录方式灵活，但没有统一 schema 定义，后续解析需手动校验
- 轨迹录制依赖人工输入与 `input()` 阻塞，不适合高速连续采集
- 目录路径与外参路径写死，移植性差

---

## 四、更深层次的架构与实现分析

### 1. 数据流对比

#### franka-gello

- 传感器 > ROS2 话题
- 话题同步 > `message_filters`
- 缓存队列 > worker 处理
- 数据写盘 > `mp4` + `parquet`

这个流程强调「在采集时即同步与结构化」，适合实时感知数据收集。

#### tavp_data_collect

- 人工示教轨迹 > JSON 轨迹
- 轨迹重放 > 机器人动作
- 每步采集视觉与点云 > 文件系统保存

这个流程强调「动作与视觉共同构建 episode」，适合多模态任务数据集创建。

### 2. 同步策略对比

- `franka-gello` 依赖 ROS topic 时间戳同步，适合纯传感器流数据
- `tavp_data_collect` 依赖轨迹时间步结构，采集时动作先/后顺序更关键

若要提高同步准确性，建议：

- 对 `franka-gello` 加入图像帧序号或 ROS 时间戳一致性校验
- 对 `tavp_data_collect` 明确“采集时间点与执行时间点”的映射

### 3. 数据可扩展性与训练准备

#### franka-gello 的优势

- Parquet 可直接用于数据分析、Pandas 等工具
- 视频序列可快速可视化
- 有同步偏差字段，便于做同步质量筛选

#### tavp_data_collect 的优势

- 结构化 episode 目录便于分割训练与验证集
- 有 RGB / 深度 / 点云，多模态数据丰富
- 轨迹文件可复用为动作监督信号

### 4. 可改进点

#### franka-gello

- 增加更稳健的序列号绑定和帧丢失处理
- 支持直接保存单帧图片或 `npz` 数据，方便离线访问
- 将 `BASE_DIR` 与话题名参数化
- 适配 ROS2 `sensor_msgs/CameraInfo` 以保留相机内参

#### tavp_data_collect

- 轨迹录制与重放统一格式，避免 `cartesian_pose` 与 `joint_position` 冲突
- 将采集流程改成“先移动到点，再采集观测”更符合时序一致性
- 增加采集调度与自动化，而非人工键盘输入
- 为 JSON / NPY 数据定义 schema/protocol，减少解析歧义

---

## 五、结论与建议

### `franka-gello` 适用场景

- 需要快速搭建 ROS2 真实机器人 + 相机流采集
- 关注在线数据采集、同步性以及传感器流下的状态关系
- 希望输出标准化 Parquet 数据用于分析

### `tavp_data_collect` 适用场景

- 需要通过示教与重放构建任务数据集
- 需要 RGB + 深度 + 点云级别的多模态数据
- 需要生成 episode 结构化数据供机器人任务学习

### 最佳组合建议

- 若要构建完整真机数据集，可将两者结合：
  1. 用 `franka-gello` 做高频同步传感器流采集
  2. 用 `tavp_data_collect` 做任务轨迹 + 视觉场景采集
- 进一步建议统一数据 schema，并引入统一路径与参数配置层

---

## 六、建议后续工作

1. 补充工程级 README、配置文件与路径参数化
2. 将 `franka-gello` 的 `BASE_DIR` 与 `task_name` 改成命令行参数
3. 为 `tavp_data_collect` 增加自动采集脚本，避免人工 `input()` 阻塞
4. 对两套数据格式做一次统一的中间表示设计，便于后续模型训练
5. 增加测试/健康检查：相机是否在线、机器人是否复位、数据写盘是否成功
