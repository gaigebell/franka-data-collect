# Dual Camera Collector 问题分析与重构设计

## 一、当前系统架构

### 1.1 硬件组成

| 设备 | 驱动方式 | 用途 |
|------|----------|------|
| Franka FR3 机械臂 | ROS 2 + franka_ros2 | 提供末端位姿、关节状态、夹爪状态 |
| Gello 遥操器 | ROS 2 topic `gello/joint_states` | 提供遥操关节角度 |
| Orbbec 深度相机 | pyorbbecsdk (直接调用，不走 ROS) | 第三视角彩色/深度图 |
| RealSense D435i | pyrealsense2 (直接调用，不走 ROS) | 手腕视角彩色/深度图 |

### 1.2 当前线程模型

```
┌─────────────────────────────────────────────────────────┐
│                  main thread                             │
│              MultiThreadedExecutor                        │
│  ┌──────────────────────────────────────────────────┐    │
│  │  _process_data 定时器 (30fps)                    │    │
│  │  - 从 camera_queue 取帧                          │    │
│  │  - 调用 _get_robot_state()                       │    │
│  │  - 调用 writer.write_frame() ← 阻塞 0.06-0.4s   │    │
│  └──────────────────────────────────────────────────┘    │
│                                                          │
│  ┌──────────────────────────────────────────────────┐    │
│  │  ROS 回调 (各独立线程)                           │    │
│  │  - _on_pose        → 更新 current_pose          │    │
│  │  - _on_gello_joints → 更新 current_gello_joints │    │
│  │  - _on_gripper     → 更新 current_gripper       │    │
│  │  (所有回调共用 _state_lock)                      │    │
│  └──────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│            CameraCaptureThread (独立线程)                │
│  - 30fps 轮询两个相机                                   │
│  - 将 frames 放入 camera_queue                          │
│  - 写入操作不经过 ROS executor                          │
└─────────────────────────────────────────────────────────┘
```

### 1.3 数据流

```
ROS Topic                    Camera SDK          User Input
    │                            │                    │
    ▼                            ▼                    ▼
/franka_robot_state_broadcaster/current_pose    遥操器
    │                            │                    │
    ▼                            ▼                    ▼
_on_pose callback          get_frame()        gello/joint_states
    │                            │                    │
    ▼                            ▼                    ▼
current_pose (lock)        camera_queue  ◄── _on_gello_joints
    │                            │                    │
    │                            ▼                    ▼
    │                     CameraCaptureThread    current_gello_joints
    │                            │                    │
    ▼                            ▼                    ▼
_process_data timer  ───►  _get_robot_state()  ◄─┘
    │                            │
    ▼                            ▼
writer.write_frame()      拼接成 robot_state dict
    │                            │
    ▼                            ▼
写入磁盘 (阻塞)        返回 {position, quaternion, gripper, gello_joints}
```

---

## 二、问题现象总结

### 2.1 观察到的现象

| 现象 | 发生时机 | 证据 |
|------|----------|------|
| `[POSE CALLBACK]` 在录制时停止打印 | 按 `s` 后 | debug.log 中 recording=True 后回调消失 |
| `[GET_ROBOT_STATE]` 数值不变 | 录制过程中 | 连续多帧 pos=(0.6276,0.1304,0.4435) 完全相同 |
| `[PROCESS_DATA]` write 耗时 0.06-0.4s | 每次写入 | debug.log 显示 get=0.0001s, write=0.1~0.4s |
| 录制结束后 `[POSE CALLBACK]` 恢复 | 按 `e` 后 | 数值开始跟随机械臂变化 |
| `ros2 topic echo` 显示数据正常 | 任意时刻 | topic 数据随机械臂运动变化 |

### 2.2 关键矛盾

- **现象 A**：`ros2 topic echo /franka_robot_state_broadcaster/current_pose` 在录制时数据持续变化
- **现象 B**：`[GET_ROBOT_STATE]` 读取的 position 数值完全不变
- **矛盾**：topic 有新数据，但代码读取不到

### 2.3 可能的根因分析

#### 假设 1：ROS 回调被 `write_frame()` 阻塞

当前 `write_frame()` 在 executor 线程（定时器所在线程）执行，而 MultiThreadedExecutor 的回调也在独立线程执行。理论上回调不应被阻塞，但实际观察是录制时回调停止。

**可能机制**：虽然回调在独立线程，但 `self.current_pose` 更新时需要 `self._state_lock`，如果 `_process_data` 长时间持有锁（整个 `write_frame()` 期间），回调会被阻塞直到锁释放。

#### 假设 2：`write_frame()` 阻塞导致队列积压

- write 耗时 0.06-0.4s
- 定时器每 33ms 触发一次 `_process_data`
- 如果 write 时间 > 33ms，下一次 `_process_data` 执行时前一次还没完成
- 这会导致什么？队列积压、帧堆积，但这不应该导致 position 不变

#### 假设 3：消息对象引用被复用

ROS 2 消息对象可能被 middleware 复用，如果 `current_pose` 持有的是对象引用而非深拷贝，后续消息到达时原对象内容被修改。

**已尝试修复**：加了 `threading.Lock` 保护，但没解决。

#### 假设 4：定时间隔内读取到的 `current_pose` 是同一时刻的旧数据

`_process_data` 每 33ms 读取一次 `current_pose`，但：
- 如果 `_on_pose` 更新了 `current_pose`
- 然后 `_process_data` 读取 `current_pose`
- 这时 `_on_pose` 又收到新消息...

这个竞态条件理论上被 lock 解决了。

---

## 三、重构设计方案

### 3.1 设计目标

1. **相机采集独立于 ROS 控制**：相机线程只负责图像采集，不阻塞 ROS 回调
2. **数据写入独立于主线程**：写入操作在独立线程执行，不阻塞数据采集
3. **状态更新不受写入影响**：ROS 回调获取最新状态不受写入操作影响
4. **保持 30fps 采集**：所有帧都带最新的机器人状态

### 3.2 目标架构

```
┌─────────────────────────────────────────────────────────────────┐
│                  main thread: rclpy MultiThreadedExecutor       │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  轻量级 ROS 回调 (只更新时间状态)                           │  │
│  │  - _on_pose        → 更新 current_pose (lock)             │  │
│  │  - _on_gello_joints → 更新 current_gello_joints (lock)    │  │
│  │  - _on_gripper     → 更新 current_gripper (lock)          │  │
│  │  这些回调绝对不能有阻塞 I/O 操作                           │  │
│  └───────────────────────────────────────────────────────────┘  │
│                            │                                    │
│                            ▼ (每帧读取当前状态)                   │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  _process_data 定时器 (30fps)                             │  │
│  │  - 从 camera_queue 取帧（不阻塞）                         │  │
│  │  - 快速获取 robot_state 快照（lock 内拷贝数值）            │  │
│  │  - 将 (frames, robot_state_snapshot) 放入 write_queue    │  │
│  │  - 总耗时应 < 1ms                                          │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│              CameraCaptureThread (独立于 executor)                │
│  - 30fps 轮询 Orbbec + RealSense 相机                           │
│  - 采集到 frames 后立即放入 camera_queue                        │
│  - 不涉及 ROS 消息传递，完全独立                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│              WriterThread (新增：独立写入线程)                    │
│  - 从 write_queue 取出 (frames, robot_state)                    │
│  - 执行所有磁盘写入操作（cv2.imwrite, imageio, parquet）        │
│  - 写入线程完全独立，不阻塞任何 ROS 操作                         │
└─────────────────────────────────────────────────────────────────┘
```

### 3.3 线程间数据传递

```
camera_queue: Queue[Frames]
    CameraCaptureThread  →  producer
    _process_data         →  consumer

write_queue: Queue[(Frames, RobotState)]
    _process_data          →  producer
    WriterThread           →  consumer
```

### 3.4 状态读取优化

当前问题之一是在 `_get_robot_state()` 中访问 `current_pose` 对象内部字段时可能存在竞态。改进方案：

```python
def _get_robot_state_snapshot(self) -> RobotStateSnapshot:
    """在锁内拷贝所有需要的数据，锁外只使用拷贝的数据"""
    with self._state_lock:
        if self.current_pose is None:
            position = (0.0, 0.0, 0.0)
            quaternion = (0.0, 0.0, 0.0, 1.0)
        else:
            position = (
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            )
            quaternion = (
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w
            )

        if self.current_gripper is None:
            gripper_positions = []
        else:
            gripper_positions = list(self.current_gripper.position)

        if self.current_gello_joints is None:
            gello_positions = [0.0] * 7
        else:
            gello_positions = list(self.current_gello_joints.position)

        return RobotStateSnapshot(
            position=position,
            quaternion=quaternion,
            gripper_positions=gripper_positions,
            gello_positions=gello_positions
        )
    # 锁外只有拷贝数据，绝不回溯到 self.current_pose 等对象
```

### 3.5 新增 WriterThread

```python
class WriterThread(threading.Thread):
    def __init__(self, write_queue: queue.Queue, writer: LeRobotWriter):
        super().__init__(daemon=True)
        self.write_queue = write_queue
        self.writer = writer
        self.running = True

    def run(self):
        while self.running:
            try:
                frames, robot_state = self.write_queue.get(timeout=0.1)
                self.writer.write_frame(frames, robot_state)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[WriterThread] 写入错误: {e}")

    def stop(self):
        self.running = False
```

### 3.6 理论分析：能否解决当前问题

| 问题 | 原因 | 新架构能否解决 | 说明 |
|------|------|---------------|------|
| 录制时 `[POSE CALLBACK]` 停止 | write_frame() 阻塞导致回调被阻塞或锁竞争 | **能** | 回调只做状态更新，无 I/O；写入在独立线程 |
| `[GET_ROBOT_STATE]` 数值不变 | write_frame() 阻塞期间 ROS 回调无法更新状态 | **能** | 回调完全不受写入影响 |
| write 耗时 0.06-0.4s | cv2.imwrite + imageio + parquet 写入太慢 | **能** | 写入在独立 WriterThread，主线程不受影响 |
| 多线程竞态 | 锁竞争导致状态被覆盖 | **能** | 新架构中 `_get_robot_state` 只在锁内做数据拷贝，耗时长操作全部在锁外 |

---

## 四、实现计划

### 4.1 代码改动概述

1. **新增 `WriterThread` 类**（新文件）
2. **修改 `CameraCaptureThread`**（当前代码基本保持不变）
3. **修改 `DualCameraCollectorNode`**：
   - 新增 `write_queue`
   - 新增 `writer_thread`
   - 修改 `_process_data`：只负责采集状态快照并放入 write_queue
   - 修改 `_get_robot_state`：返回 snapshot 对象而非操作 ROS 消息
   - ROS 回调保持极简（只更新状态）
4. **修改 `LeRobotWriter`**：基本保持不变（write_frame 逻辑不变）

### 4.2 关键设计决策

1. **为何不改变相机线程**：相机 SDK 调用（pyorbbecsdk/pyrealsense2）已经是独立线程，只是不走 ROS。当前架构这点没问题。

2. **为何写入要独立线程**：因为 `cv2.imwrite` 和 `imageio` 是磁盘 I/O 操作，可能阻塞数毫秒到数百毫秒。在 30fps（每帧 33ms）场景下，0.4s 的写入会丢失 12 帧。

3. **为何回调要极简**：ROS 回调如果涉及任何耗时操作，会阻塞该回调所在的线程，导致消息队列积压。理想情况下回调只做状态更新（毫秒级），数据消费完全异步。

### 4.3 性能预估

| 操作 | 当前耗时 | 目标耗时 | 说明 |
|------|----------|----------|------|
| `_process_data` 一次迭代 | 0.06-0.4s | < 1ms | 只做队列取帧 + 状态拷贝 |
| ROS 回调 `_on_pose` | < 1ms | < 1ms | 只做状态更新 |
| `write_frame` | 0.06-0.4s | 不限（异步） | WriterThread 独立处理 |
| 每帧端到端延迟 | 33ms + 写入阻塞 | ~33ms + 写入排队 | 写入不阻塞采集 |

---

## 五、现有问题 vs 新架构对照

| 现有问题 | 根因 | 新架构处理方式 |
|----------|------|----------------|
| 录制时 pose 不变 | write_frame 阻塞 ROS 回调，状态无法更新 | 写入线程独立，回调不受影响 |
| write 耗时不稳定 (0.06-0.4s) | 图像写入 + 视频写入 + parquet 缓冲 | WriterThread 异步处理，不阻塞主循环 |
| 调试信息刷屏 | print 阻塞 I/O | 写入线程无 print，主线程 print 极少 |
| 多线程竞态条件 | 锁持有期间访问 msg 内部字段 | 锁内只做数值拷贝，锁外不访问 ROS 对象 |