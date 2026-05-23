# 调试记录与 Bug 修复

## 一、dual_camera_collect 问题记录

### Bug 1: 录制时机械臂状态停滞

**发现时间**：2025-05
**现象**：
- `ros2 topic echo` 正常显示状态变化
- 录制的视频帧对应的状态数据完全不变
- `[POSE CALLBACK]` 在录制时停止打印

**排查过程**：
1. 检查 ROS topic → 正常
2. 添加 debug.log → 发现 write 耗时 0.06~0.4s
3. 分析 `_process_data` 线程 → 定位到写入阻塞

**根因**：
- `_process_data` 在 executor 线程执行 `write_frame()`
- 写入操作持有 `_state_lock`，阻塞了 ROS 回调
- 回调无法更新状态，导致状态停滞

**修复方案**：
- 引入 WriterThread 独立线程处理写入
- `_process_data` 只负责取帧和入队，不做 I/O

**状态**：已修复 ✓

---

### Bug 2: 停止录制时数据丢失

**发现时间**：2025-05
**现象**：
- 按下停止键后视频文件损坏
- parquet 数据不完整

**根因**：
- `WriterThread.stop()` 直接设置 `running=False`
- 队列中残留数据被丢弃

**修复方案**：
- 排空模式：设置 `_stopping=True`
- 等待队列清空后再退出

```python
def stop(self):
    self._stopping = True  # 进入排空模式
```

**状态**：已修复 ✓

---

### Bug 3: Orbbec 相机 MJPG 解码失败

**发现时间**：2025-05
**现象**：
- 4K 模式下图像为空白或花屏

**根因**：
- Orbbec 4K 流使用 MJPG 格式
- 解码方式错误

**修复方案**：
```python
if color_format == OBFormat.MJPG:
    color_bgr = cv2.imdecode(color_data, cv2.IMREAD_COLOR)
```

**状态**：已修复 ✓

---

### Bug 4: 写入视频帧尺寸不匹配

**发现时间**：2024-11
**现象**：
- `VideoWriter.write()` 报错尺寸不匹配

**根因**：
- 实际帧尺寸与初始化时参数不一致

**修复方案**：
```python
def write_video_frame(self, bgr_image):
    if bgr_image.shape != expected_shape:
        bgr_image = cv2.resize(bgr_image, (width, height))
    self.video_writer.write(bgr_image)
```

**状态**：已修复 ✓

---

## 二、tavp_data_collect 问题记录

### Bug 1: 轨迹重放时夹爪状态不同步

**发现时间**：2024-11
**现象**：
- 夹爪在轨迹执行过程中状态异常

**根因**：
- `is_grasped` 全局变量在多步轨迹中状态混乱

**修复方案**：
```python
def replay_trajectory(...):
    global is_grasped
    is_grasped = False  # 重置状态
```

**状态**：已修复 ✓

---

### Bug 2: wait_for_frames 返回空帧

**发现时间**：2024-11
**现象**：
- 程序卡死或异常退出

**根因**：
- 相机硬件或 USB 带宽问题导致丢帧

**修复方案**：
```python
max_retry = 20
for attempt in range(max_retry):
    raw_frameset = self.pipeline.wait_for_frames(timeout_ms)
    if raw_frameset:
        break
    print(f"警告: 重试 {attempt+1}/{max_retry}")
```

**状态**：已修复 ✓

---

### Bug 3: 点云坐标转换错误

**发现时间**：2024-11
**现象**：
- 点云在世界坐标系下位置不对

**根因**：
- 外参矩阵使用错误

**修复方案**：
- 确保使用正确的 `T_base_camera` 外参
- 验证：`python -c "import numpy as np; print(np.load('T_base_orbbec_1119.npy').shape)"`

**状态**：已修复 ✓

---

## 三、franka-gello 问题记录

### Bug 1: 多线程写入竞争

**发现时间**：2024-10
**现象**：
- 视频文件损坏
- Parquet 数据丢失

**根因**：
- worker 线程和主线程同时写入

**修复方案**：
```python
# 使用锁保护
with self.frame_buffer_lock:
    self.frame_buffer.append(img_np)
with self.buffer_lock:
    self.meta_buffer[idx] = ...
```

**状态**：已修复 ✓

---

### Bug 2: 录制停止后内存溢出

**发现时间**：2024-10
**现象**：
- 长时间录制后内存占用过高

**根因**：
- `frame_buffer` 无限增长

**修复方案**：
- 预分配固定大小的 `np.zeros()`
- 超出 buffer_size 时停止录制

**状态**：已修复 ✓

---

## 四、经验总结

### 4.1 调试方法论

1. **建立基准**：用 `ros2 topic echo` 确定 topic 正常
2. **添加日志**：结构化日志帮助定位问题
3. **绘制数据流**：画出数据流图找瓶颈
4. **隔离变量**：逐一排除可能原因

### 4.2 常见陷阱

| 陷阱 | 表现 | 解决方案 |
|------|------|----------|
| I/O 在主线程 | 状态卡住 | 异步写入线程 |
| 深拷贝遗漏 | 数据错乱 | 使用 copy.deepcopy |
| 锁内耗时 | 回调阻塞 | 锁内只做拷贝 |
| 队列无限增长 | 内存溢出 | 设置 maxsize |

### 4.3 预防措施

1. **代码审查**：检查是否有耗时操作在回调/主线程
2. **日志记录**：关键路径要有日志
3. **超时机制**：所有 I/O 操作要有超时
4. **边界检查**：队列满、buffer 满等边界情况要处理

---

## 五、测试检查清单

每次修改代码后，检查：

- [ ] 相机能否正常初始化
- [ ] 录制开始/停止是否正常
- [ ] 数据文件是否完整（视频 + parquet）
- [ ] parquet 数据能否正确读取
- [ ] 多 episode 连续录制是否正常
- [ ] 长时间录制是否稳定
- [ ] 内存占用是否正常
- [ ] 磁盘写入是否正常