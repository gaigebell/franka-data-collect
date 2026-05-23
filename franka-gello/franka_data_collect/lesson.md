# franka-gello 项目经验总结

## 一、项目背景

### 1.1 目标
构建一个基于 ROS 2 + Gello 的实时数据采集系统：
- 订阅机械臂状态话题
- 订阅 RealSense 相机图像话题
- 同步采集图像 + 状态，写入 MP4 + Parquet

### 1.2 技术选型
- 机械臂控制：ROS 2 + franka_ros2 + Gello
- 相机：RealSense via ROS topic (`/camera/camera/color/image_rect_raw`)
- 数据同步：`message_filters.ApproximateTimeSynchronizer`
- 数据保存：MP4 (imageio) + Parquet

---

## 二、核心问题与解决

### 问题 1：同步策略

**设计选择**：`ApproximateTimeSynchronizer`

```python
self.ts = message_filters.ApproximateTimeSynchronizer(
    [sub_image, sub_pose, sub_gripper],
    queue_size=10,
    slop=0.1  # 允许 0.1 秒的时间差
)
```

**参数选择逻辑**：
- `slop=0.1`：允许话题时间戳相差 100ms 以内视为"同步"
- `queue_size=10`：如果同步失败，最多缓存 10 条消息

**为什么不是精确同步**：
- ROS 话题本身就是异步的，时间戳不可能完全一致
- 机械臂状态频率 ~1kHz，图像 ~30fps，时间基准不同
- 100ms 的误差在数据采集场景下是可接受的

### 问题 2：内存缓冲设计

**设计选择**：预分配 numpy 数组

```python
DTYPE_META = np.dtype([
    ('img_sec', np.int32), ('img_nsec', np.int32),
    ('pose_sec', ...), ('grip_sec', ...),
    ('x', np.float64), ('y', np.float64), ...
])
self.meta_buffer = np.zeros(buffer_size, dtype=DTYPE_META)
```

**优势**：
- 预分配避免频繁 `append()` 导致的内存碎片
- 结构化数组（Structured Array）保证类型安全
- 固定长度 `buffer_size=10000` 防止无限内存增长

### 问题 3：写入时机

**设计选择**：录制停止时批量写入

```python
def save_data(self):
    frames_to_save = []
    with self.buffer_lock:
        # 一次性拷贝所有数据
        valid_data = self.meta_buffer[:count].copy()
        frames_to_save = self.frame_buffer[:]
        self.frame_buffer.clear()
```

这比边采集边写入更高效，因为：
- 视频文件需要连续写入
- 批量写入可以复用编码器的内部缓冲区

---

## 三、架构设计

### 3.1 线程模型

```
ROS MultiThreadedExecutor
    │
    ├── sync_callback ─────────────────────────► data_queue
    │   (ApproximateTimeSynchronizer)              │
    │                                              ▼
    │                                      _worker_loop
    │                                            │
    │                                            ▼
    │                                     frame_buffer
    │                                     meta_buffer  ──► 磁盘写入
    │
    └── 键盘监听 (独立线程)

```

### 3.2 数据流

```
ROS Topic (/camera/..., /franka_robot_state_broadcaster/...)
        │
        ▼
ApproximateTimeSynchronizer (slop=0.1s)
        │
        ▼
sync_callback ──► data_queue
        │
        ▼
_worker_loop (后台线程)
        │
        ├── FPS 控制 (20fps)
        ├── 解析图像
        └── 写入 buffer
```

### 3.3 Parquet Schema

```
observation.state:
  - img_stamp_sec/nsec: 图像 ROS 时间戳
  - pose_stamp_sec/nsec: 位姿 ROS 时间戳
  - grip_stamp_sec/nsec: 夹爪 ROS 时间戳
  - sync_diff_pose_ms: 图像和位姿的时间差 (ms)
  - sync_diff_grip_ms: 图像和夹爪的时间差 (ms)
  - pos_x/y/z: 末端位置 (m)
  - ori_x/y/z/w: 末端四元数
  - gripper_1/2: 夹爪两个手指位置 (m)
```

---

## 四、经验教训

### 4.1 同步近似的代价

**教训**：`slop=0.1s` 的同步误差是**不可忽略**的。

对于 30fps 视频，每帧约 33ms。如果同步误差 ~100ms，意味着图像和状态的对应关系可能差 3 帧。

**改进方向**：
- 减小 slop（如 0.03s）
- 增加图像帧号或状态帧号作为辅助同步依据
- 在保存时记录实际的同步误差，供后处理筛选

### 4.2 内存映射的陷阱

**教训**：numpy 结构化数组的切片是**视图**，不是拷贝。

```python
valid_data = self.meta_buffer[:count].copy()  # 必须 copy()
```

如果不加 `.copy()`，后续清空 buffer 会影响 `valid_data`。

### 4.3 队列积压的处理

**设计**：`queue.Queue(maxsize=2048)`

当队列满时，`put(packet, block=False)` 会抛出 `queue.Full`。此时：
- 旧数据被丢弃（最新数据无法插入）
- 这会导致**丢帧**，但不会崩溃

更好的设计可能是：
- 阻塞等待一小段时间
- 或者丢掉最老的数据（滑动窗口）

---

## 五、与计算机科学基础课程的联系

### 5.1 操作系统 - 线程与同步

**线程模型**：
- `DataCollectorNode`：主线程，运行在 MultiThreadedExecutor
- `_worker_loop`：后台工作线程
- `keyboard_listener`：键盘监听线程

**同步原语**：
- `threading.Lock`：保护 `meta_buffer` 和 `frame_buffer`
- `queue.Queue`：线程间通信，替代直接共享内存

### 5.2 数据结构 - 队列

**为什么用 Queue 而不是直接共享列表**：
- `Queue` 是**线程安全**的，push/pop 操作是原子性的
- 直接共享列表需要手动加锁，容易出错
- `Queue.maxsize` 防止无限内存增长

### 5.3 计算机网络 - 时间同步

**ROS 话题的时间戳同步**：
- 每个 ROS 消息带有 `header.stamp`（ROS 时间）
- `ApproximateTimeSynchronizer` 基于时间戳做同步

**为什么不用序列号**：
- 图像和机械臂状态是不同的 topic，没有统一序列号
- 只能依赖时间戳做近似对齐

### 5.4 数据库 - Parquet 列式存储

**Parquet 优势**：
- **列式存储**：读取时只需读取需要的列
- **压缩友好**：相同类型的值连续存储，压缩率高
- **类型安全**：schema 定义了每列的类型

### 5.5 软件工程 - 设计模式

**观察者模式**：
- ROS subscriber 是观察者
- 话题是主题
- 回调函数处理消息

**生产者-消费者模式**：
- 回调生产数据 → 放入队列
- worker 消费数据 → 写入磁盘

---

## 六、面试官灵魂拷问（模拟）

### 环节一：ROS 同步

**面试官**：`ApproximateTimeSynchronizer` 的 slop 参数是怎么选的？100ms 有什么依据？

**参考答案**：
slop 的选择是**延迟容忍度和同步成功率**的权衡。

当时配置的是 `slop=0.1`（100ms），考虑的是：

1. **ROS 话题延迟**：机械臂状态 ~1kHz，图像 ~30fps，两者的发布频率不同，延迟自然会有波动

2. **系统负载**：实际运行时可能有 CPU 调度延迟，偶尔的 100ms 延迟是可接受的

3. **实际测试**：通过 `load_data.py` 分析同步差分布，发现大多数差值在 50ms 以内，100ms 可以覆盖 95% 的情况

**如果 slop 太小**：
- 同步成功率低，很多帧被丢弃
- 有效数据减少

**如果 slop 太大**：
- 同步精度差，状态和图像可能对应不上
- 100ms 已经很大了，33ms 的帧间隔意味着可能差了 3 帧

---

**面试官**：为什么不用精确时间同步？ApproximateTime 和 ExactTime 有什么区别？

**参考答案**：
精确时间同步（`ExactTimeSynchronizer`）要求所有消息的**时间戳完全相同**，这在实际系统中几乎不可能。

原因：
1. **时钟源不同**：相机、IMU、机械臂可能各有自己的时钟源
2. **传输延迟不同**：不同 topic 走的路径不同
3. **发布频率不同**：1kHz 和 30fps 根本无法对齐到同一时刻

`ApproximateTimeSynchronizer` 的策略是：
- 找时间戳最接近的消息组
- 允许一定时间差（slop）
- 这是工业界常用的实用方案

---

### 环节二：内存管理

**面试官**：你用 numpy 预分配 buffer，而不是 `append()` 到 list，这两者有什么区别？

**参考答案**：
这是**动态数组 vs 预分配数组**的区别。

**list + append**：
```python
frame_buffer = []
frame_buffer.append(img_np)  # 每次 append 可能触发内存重新分配
```
- 优点：灵活，内存随数据量增长
- 缺点：频繁 `append()` 会导致内存碎片化

**预分配 numpy 数组**：
```python
meta_buffer = np.zeros(buffer_size, dtype=DTYPE_META)
row = meta_buffer[idx]  # 直接索引到固定位置
```
- 优点：内存连续，索引速度快
- 缺点：固定大小，超出后需要处理

**为什么选择预分配**：
- 数据采集的帧数是可控的（录制时长 × fps）
- 预分配可以避免 `append()` 的性能开销
- 固定大小可以防止内存无限增长

---

**面试官**：如果录制数据超过 buffer_size 怎么办？

**参考答案**：
当前实现会**停止录制**：

```python
if self.buffer_index >= self.max_buffer_size:
    self.is_recording = False
    continue
```

这是一个保守的设计——宁可丢数据，也不能覆盖旧数据。

**更好的方案**：
1. **环形缓冲区**：覆盖最老的数据（新数据覆盖旧数据）
2. **动态扩展**：检测到快满了就扩容
3. **分段存储**：一个 buffer 满了就开新的，最后合并

当时选择"停止录制"是因为数据采集场景下，丢数据的代价比丢帧更严重——录了一半的数据可能无法使用。

---

### 环节三：队列设计

**面试官**：你的 `data_queue` 和 `frame_buffer` 是两个独立的 buffer，为什么不合成一个？

**参考答案**：
因为它们的数据特性和生命周期不同。

**data_queue（队列）**：
- 传递的是**消息对象**（ROS msg）
- 生存周期短，worker 处理完就可以丢弃
- 需要线程安全，用 `Queue` 方便

**frame_buffer（列表）**：
- 存储的是**numpy 数组**（图像数据）
- 保留到 `save_data()` 时才释放
- 需要和 meta_buffer 对齐（长度一致）

**为什么要对齐**：
```python
# worker 循环中
with self.frame_buffer_lock:
    self.frame_buffer.append(img_np)  # 图像单独存

with self.buffer_lock:
    # 状态存在结构化数组中
    row = self.meta_buffer[idx]
```

如果合并成一个，每次需要同时锁住图像和状态两个字段，粒度太粗。而且图像很大（MB级别），不应该和轻量的状态数据混合管理。

---

### 环节四：视频编码

**面试官**：为什么用 imageio 而不是 cv2.VideoWriter？

**参考答案**：
因为 imageio 的 `mimwrite` 更适合批量写入。

**cv2.VideoWriter**：
```python
writer = cv2.VideoWriter(...)
for frame in frames:
    writer.write(frame)  # 边处理边写入
```
- 需要手动管理 writer 的生命周期
- 每帧单独写入

**imageio.mimwrite**：
```python
imageio.mimwrite(video_path, frames, fps=self.fps)
```
- 一次性写入所有帧
- 内部有缓冲机制，批量编码效率更高

**选择 imageio 的原因**：
- worker 线程最后一次性写入，不需要边采集边写入
- `mimwrite` 调用 ffmpeg 编码，效率比 cv2 高
- mp4 格式支持更好（cv2 的 mp4v codec 在某些平台有问题）

---

### 环节五：架构对比

**面试官**：你的项目和 dual_camera_collect 有什么区别？为什么不能直接用你的架构？

**参考答案**：
两个项目有不同的设计目标。

**franka-gello 的架构**：
- 单一相机（RealSense via ROS topic）
- `ApproximateTimeSynchronizer` 做同步
- worker 线程同步写入

**dual_camera_collect 的架构**：
- 双相机（Orbbec + RealSense 各自 SDK）
- 多线程解耦（采集线程 + 写入线程）
- 双缓冲队列

**为什么 dual_camera_collect 不能直接用我的架构**：
1. **相机数量**：双相机需要两个独立的数据源，我的 single-camera 架构不支持
2. **同步精度**：我的 ApproximateTimeSynchronizer 在双相机场景下扩展性差
3. **性能**：双相机 30fps × 2 的数据量，单 worker 可能成为瓶颈

**两个架构的共同问题**：
- 都是"录制停止时一次性写入"，如果崩溃会丢数据
- 都没有视频流的实时预览

---

## 七、可复用的经验模板

### 7.1 ROS 同步订阅模板

```python
import message_filters
from sensor_msgs.msg import Image, PoseStamped, JointState

sub_image = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
sub_pose = message_filters.Subscriber(self, PoseStamped, '/pose')
sub_gripper = message_filters.Subscriber(self, JointState, '/gripper')

ts = message_filters.ApproximateTimeSynchronizer(
    [sub_image, sub_pose, sub_gripper],
    queue_size=10,
    slop=0.1  # 100ms
)
ts.registerCallback(self.sync_callback)
```

### 7.2 预分配 buffer 模板

```python
import numpy as np

DTYPE = np.dtype([
    ('field1', np.float64),
    ('field2', np.int32),
])

buffer_size = 10000
buffer = np.zeros(buffer_size, dtype=DTYPE)
buffer_index = 0
buffer_lock = threading.Lock()

def on_data(data):
    global buffer_index
    with buffer_lock:
        if buffer_index >= buffer_size:
            return False  # buffer 满
        buffer[buffer_index]['field1'] = data.value1
        buffer[buffer_index]['field2'] = data.value2
        buffer_index += 1
    return True
```

### 7.3 Parquet 保存模板

```python
import pyarrow as pa
import pyarrow.parquet as pq

schema = pa.schema([
    ('col1', pa.float64()),
    ('col2', pa.int32()),
])

table = pa.table({
    'col1': pa.array(data['col1']),
    'col2': pa.array(data['col2']),
}, schema=schema)

pq.write_table(table, 'output.parquet')
```

---

## 八、总结

`franka-gello` 的核心经验：

1. **同步是近似不是精确**：100ms 的 slop 是工程权衡，不是理论最优
2. **预分配比动态增长好**：避免内存碎片，索引速度快
3. **队列解耦生产者消费者**：Queue 是线程间通信的好工具
4. **批量写入效率更高**：边采集边写入不如最后一次性写入

这个项目是三个中最基础的，但它教会我们：ROS 环境下的数据采集，**同步是核心问题**，所有设计都围绕这个问题展开。