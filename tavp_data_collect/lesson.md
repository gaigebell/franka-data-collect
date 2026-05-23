# tavp_data_collect 项目经验总结

## 一、项目背景

### 1.1 目标
构建一个"轨迹示教 + 重放采集"的数据采集系统：
- 人工示教录制关节轨迹
- 重放轨迹时同步采集视觉数据（RGB + 深度 + 点云）
- 适用于需要精确动作序列的任务数据集

### 1.2 技术选型
- 机械臂控制：franky Python SDK（直接 IP 连接，不走 ROS）
- 相机：Orbbec 直接 SDK（pyorbbecsdk）
- 数据格式：PNG + NPY + JSON
- 轨迹文件：JSON 格式

---

## 二、核心问题与解决

### 问题 1：采集与移动的时序问题

**现象**：
- `replay_keypoint_orbbec_v3.py` 中先调用 `take_snapshot()`，再执行 `JointMotion`
- 这导致采集到的状态和执行的动作用于不同时间点

**分析**：
```python
# 当前顺序
take_snapshot(robot, gripper, camera, episode_dir, i)  # 采集此刻状态
robot.move(motion)  # 然后移动到下一点
```

采集时机械臂还没移动到目标位置，但采集的数据被标记为该目标时间步的状态。

**影响**：
- 状态和动作错位一个时间步
- 不适合需要精确状态-动作对应的强化学习训练

### 问题 2：相机的重试机制

**现象**：
- `wait_for_frames()` 有时会返回 `None`
- 连续失败导致程序崩溃

**解决**：
```python
max_retry = 20
for attempt in range(max_retry):
    raw_frameset = self.pipeline.wait_for_frames(timeout_ms)
    if raw_frameset:
        break
    print(f"警告: 丢失帧, 重试 {attempt+1}/{max_retry}...")
```

### 问题 3：外参矩阵的使用

**现象**：
- 外参 `T_base_camera` 存储为 4x4 矩阵
- 点云转换时需要正确处理齐次坐标

**实现**：
```python
def transform_point_cloud_to_world(self, pcd_camera: np.ndarray) -> np.ndarray:
    T_base_camera = self.extrinsics
    points_camera_flat = pcd_camera.reshape(-1, 3)
    ones = np.ones((points_camera_flat.shape[0], 1), dtype=points_camera_flat.dtype)
    points_camera_homogeneous = np.hstack((points_camera_flat, ones))
    result_homogeneous_T = T_base_camera @ points_camera_homogeneous.T
    points_world_flat = result_homogeneous_T.T[:, :3]
    return points_world_flat.reshape(height, width, 3)
```

---

## 三、架构设计

### 3.1 主循环结构

```
load_trajectory(JSON)
        ↓
for waypoint in trajectory:
    │
    ├── take_snapshot() ──► 保存 RGB + 深度 + 点云 + 状态 JSON
    │
    ├── JointMotion(target_q) ──► 机械臂移动到目标关节角度
    │
    ├── grasp()/open() ──► 夹爪动作
    │
    └── sleep(0.3)
        │
        ↓
take_snapshot() ──► 保存最后一个状态
```

### 3.2 数据目录结构

```
episode_X/
├── frontImg/          # RGB 图像 PNG
│   └── {timestep}.png
├── frontDeep/         # 深度图 NPY (毫米)
│   └── {timestep}.npy
├── frontPcd/          # 点云 NPY (世界坐标系, 米)
│   └── {timestep}.npy
└── observation/        # 状态 JSON
    └── {timestep}.json
```

---

## 四、经验教训

### 4.1 采集时序的重要性

**教训**：数据采集的时序直接影响数据质量。

- **强化学习要求**：`observation.state[t]` 对应 `action[t]`
- **当前实现**：`observation.state[t]` 对应 `action[t+1]`（采集在移动之前）

**正确做法**：
```python
# 方案 1：先移动到目标位置，再采集
robot.move(motion)
time.sleep(0.1)  # 等待稳定
take_snapshot()

# 方案 2：采集时记录"下一个目标位置"而非"当前位置"
# 采集的数据 label 为下一次移动的目标
```

### 4.2 相机封装的演进

**v4 → v6 的改进**：
- 支持 4K 彩色 MJPG 流
- 可选深度流（4K 模式下必须关闭深度）
- 增加帧获取重试逻辑（防止 segfault）
- 从 profile 动态读取内参

**设计原则**：相机类应该封装好所有硬件相关操作，对外只暴露"干净"的接口。

### 4.3 硬件外参的重要性

**外参矩阵 T_base_camera**：
- 定义相机坐标系到机械臂基座坐标系的变换
- 用于将相机拍摄的深度图/点云转换到机器人世界坐标系
- 通过标定得到（项目中使用的是 `T_base_orbbec_1119.npy`）

**应用场景**：
- 视觉-触觉学习：把相机看到的和机器人做的统一到同一坐标系
- 碰撞检测：在世界坐标系下检测障碍物
- 抓取姿态计算：相机中检测到的目标位置需要转换到基座坐标系

---

## 五、与计算机科学基础课程的联系

### 5.1 数据结构 - 齐次坐标与矩阵变换

点云从相机坐标系转换到世界坐标系，核心是齐次坐标变换：

```
P_world = T_base_camera × P_camera_homogeneous
```

其中 `T_base_camera` 是 4x4 矩阵：
```
| R  t |   R = 3x3 旋转矩阵
| 0 1 |   t = 3x1 平移向量
```

这对应线性代数中的**仿射变换**。

### 5.2 计算机图形学 - 针孔相机模型

从深度图生成点云使用的是**针孔相机模型**：

```python
X = (u - cx) * depth / fx
Y = (v - cy) * depth / fy
Z = depth
```

其中 `(fx, fy, cx, cy)` 是相机内参，`(u, v)` 是像素坐标。

### 5.3 软件工程 - 封装与抽象

**OrbbecCamera 类的设计**：

| 公共接口 | 说明 |
|---------|------|
| `get_frames()` | 获取彩色+深度帧 |
| `get_images_from_frames()` | 解码为 numpy 数组 |
| `get_aligned_point_cloud()` | 一键生成世界坐标系点云 |

调用者不需要知道：
- MJPG 如何解码
- 对齐 Filter 如何工作
- 内参从哪个 profile 获取

这就是**封装**——隐藏实现细节，提供简洁接口。

### 5.4 操作系统 - I/O 与进程间通信

franky SDK 通过 IP 连接机械臂，本质是**网络套接字通信**：
- Robot IP: `"172.16.0.2"`
- 使用 franky 库封装了通信协议
- 控制命令 → 机械臂执行 → 状态返回

如果 franky 通信超时或失败，机械臂可能继续执行危险动作，所以需要 `recover_robot()` 错误恢复机制。

---

## 六、面试官灵魂拷问（模拟）

### 环节一：轨迹重放与时序

**面试官**：你说到采集在移动之前执行，这具体是什么问题？

**参考答案**：
这是一个**状态-动作对齐错位**的问题。

在强化学习数据中，我们希望 `observation.state[t]` 对应机械臂执行 `action[t]` 时的状态。但当前代码：
```python
take_snapshot()  # 采集 t 时刻的观测
robot.move(motion)  # 移动到 t+1 时刻的目标
```

这意味着我们记录的状态其实是"移动前的状态"，但标签却是"移动后的目标"。两者差了一帧。

**影响**：
- 如果用这个数据训练策略网络，网络会学到"错误的映射"
- 比如当前状态 A 应该执行动作 X 到达状态 B，但实际上我们记录的是"状态 A 对应动作 Y（Y 是 B 的前一个动作）"

**修正方案**：
```python
# 正确做法：先移动，后采集
robot.move(motion)
time.sleep(0.1)  # 等待机械臂稳定
take_snapshot()  # 采集移动后的状态
```

---

**面试官**：为什么轨迹重放需要"示教"？为什么不直接记录关节角度序列然后重放？

**参考答案**：
这里涉及**轨迹示教（Teaching）**的概念。

有两种方式：
1. **示教（Playback）**：人工操作机械臂，记录运动路径
2. **编程（Programming）**：直接写关节角度序列

示教的优势：
- **自然映射**：人的手部动作直接映射到机器人动作，不需要复杂的逆向运动学
- **避障直觉**：人在操作时会本能地绕过障碍物
- **简单直接**：不需要理解机器人运动学也能录数据

对于抓取、放置等任务，示教比编程更直观。操作员可以直接看到机器人执行的动作是否合理。

---

### 环节二：相机与点云

**面试官**：你那个点云生成，为什么要用齐次坐标？

**参考答案**：
因为我们用的是**仿射变换**，不是线性变换。

相机坐标系到世界坐标系的变换包含旋转和平移：
```
P_world = R × P_camera + t
```

这不能简单地用 3x3 矩阵表示，需要用 4x4 齐次矩阵：
```
| R  t |   | P_camera |   | P_world |
| 0 1 | × |    1     | = |    1    |
```

其中 `[R|t]` 是 4x4 矩阵的前 3 行，第四行 `[0 0 0 1]` 是齐次坐标的约定。

如果不加那个 1（齐次坐标的齐次部分），3x3 矩阵只能表示纯旋转，不能加平移。

---

**面试官**：4K 模式下为什么要关闭深度流？

**参考答案**：
这是硬件和带宽的权衡。

Orbbec 相机的数据流是：
- 4K MJPG 彩色流：需要更大带宽
- 深度流：额外的数据通道

同时开启两种高分辨率流，相机的处理能力可能跟不上。4K MJPG 已经占用了大部分带宽，深度流就只能关闭。

工程上这是一个**配置问题**——不同分辨率组合有不同的能力边界。代码中通过 `enable_depth` 参数控制：

```python
if not self.enable_depth and self.enable_alignment:
    self.enable_alignment = False  # 深度关闭时强制关闭对齐
```

---

### 环节三：外参与标定

**面试官**：外参矩阵是怎么得到的？谁来标定？

**参考答案**：
外参标定是机器人视觉中的经典问题。

**标定方法**：
1. 将标定板（如棋盘格）放在相机视野内
2. 机械臂末端带着针或标记移动到不同位置
3. 同时记录：机械臂基座下的末端位姿 + 相机观测到的标定板位置
4. 用 PnP 等方法求解 `T_base_camera`

**项目中的外参**：
文件 `T_base_orbbec_1119.npy` 是提前标定好的结果。日期 1119 可能是标定日期。

**为什么需要外参**：
- 相机看到的是像素坐标
- 机械臂控制是世界坐标系
- 需要把相机看到的目标转换到机器人能执行的动作

---

### 环节四：代码质量

**面试官**：你的代码里有很多 `try-except` 然后 raise，这是什么风格？

**参考答案**：
说实话，这个风格不太好。

代码中：
```python
try:
    color_frame, depth_frame = camera.get_frames()
except RuntimeError as e:
    print(f"警告: 获取帧失败: {e}")
    raise  # 强制抛出异常
```

这样做的考虑是：
- 如果采集失败，录制的数据不完整，继续也没意义
- 打印警告后立即崩溃，避免产生难以排查的隐性问题

但更好的做法可能是：
- 在 `take_snapshot()` 内部处理重试
- 或者让调用者决定如何处理

这是**防御性编程**和**快速失败**之间的权衡。快速失败（fail fast）更适合数据采集场景——录一半的数据比完全不录更糟糕。

---

### 环节五：系统集成

**面试官**：franky 和 ROS 是怎么配合的？你们项目里哪些地方用 franky，哪些地方用 ROS？

**参考答案**：
好问题。这个项目的架构比较"混搭"。

**franky**：
- 直接 IP 连接机械臂
- 用于轨迹重放（`JointMotion`）和夹爪控制
- 不走 ROS 通信，直接 TCP/IP 协议

**ROS 2**：
- `franka-gello` 项目中，相机和控制都走 ROS
- 但在 `tavp_data_collect` 中，相机直接用 SDK，没走 ROS

**为什么这样设计**：
- `tavp_data_collect` 的目标是**示教重放**，不需要实时遥操作
- 直接控制更简单可靠，不需要 ROS 环境
- franky 提供了更底层的控制接口

两种方案各有优劣：
- ROS 方案：生态好、可视化方便、多节点协同
- franky 方案：更直接、延迟更低、依赖更少

---

## 七、可复用的经验模板

### 7.1 相机类封装模板

```python
class CameraDriver:
    def __init__(self, config):
        # 1. 初始化硬件
        self.pipeline = SDK_Pipeline()
        
        # 2. 配置流（处理不同分辨率的 fallback）
        self.color_profile = self._get_best_profile(...)
        
        # 3. 读取内参
        self.intrinsics = self._extract_intrinsics(self.color_profile)
        
        # 4. 启动
        self.pipeline.start(self.config)
    
    def get_frames(self):
        # 重试逻辑
        for attempt in range(max_retry):
            frames = self.pipeline.wait_for_frames(timeout)
            if frames:
                return self._process_frames(frames)
        raise RuntimeError("获取帧失败")
```

### 7.2 轨迹数据模板

```json
[
    {
        "timestep": 0,
        "joint_position": [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5],
        "cartesian_xyz": [0.5, 0.0, 0.3],
        "cartesian_quat_xyzw": [0.0, 1.0, 0.0, 0.0],
        "gripper_grasped": 0,
        "gripper_position": 0.04
    },
    ...
]
```

### 7.3 坐标系转换模板

```python
import numpy as np

def camera_to_world(pcd_camera, T_base_camera):
    """点云从相机坐标系转换到世界坐标系"""
    H, W, _ = pcd_camera.shape
    points_flat = pcd_camera.reshape(-1, 3)
    ones = np.ones((points_flat.shape[0], 1))
    points_homogeneous = np.hstack([points_flat, ones])
    points_world = (T_base_camera @ points_homogeneous.T).T[:, :3]
    return points_world.reshape(H, W, 3)
```

---

## 八、总结

`tavp_data_collect` 的核心经验：

1. **时序一致性**：采集必须在动作执行之后，不能在之前
2. **硬件封装**：把 SDK 的复杂操作封装成简单的类接口
3. **坐标系变换**：理解齐次坐标和仿射变换是 3D 视觉的基础
4. **错误恢复**：硬件操作要有重试机制，不能轻易崩溃

这个项目相对简单直接，但它教会我们：数据采集的**每一步都要考虑时序**，错位的数据比没数据更危险。