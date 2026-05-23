# 硬件设施与技术环境文档

## 一、硬件平台

### 1.1 机械臂

| 属性 | 值 |
|------|-----|
| 型号 | **Franka FR3** (Franka Panda 3) |
| 控制接口 | **franky** (Python SDK via IP) |
| ROS 接口 | franka_ros2 / franka_ros2_driver |
| IP 地址 | `172.16.0.2` |
| 控制频率 | 1 kHz |
| 自由度 | 7 DOF |
| 末端执行器 | **Franka Gripper** (夹爪) |
| Gripper 控制方式 | franky.Gripper |
| Gripper 行程 | 0~0.08m (两指间距) |

### 1.2 相机

#### Orbbec Astra Pro (前置相机)

| 属性 | 值 |
|------|-----|
| 型号 | Orbbec Astra Pro |
| 连接方式 | USB 3.0 |
| SDK | pyorbbecsdk |
| 彩色分辨率 | 1280×720 @ 30fps |
| 深度分辨率 | 640×576 @ 30fps |
| 深度范围 | 0.6m ~ 8m |
| 内参获取 | 从硬件 profile 自动读取 |

#### Intel RealSense D435i (腕部相机)

| 属性 | 值 |
|------|-----|
| 型号 | Intel RealSense D435i |
| 连接方式 | USB 3.0 |
| SDK | pyrealsense2 |
| 彩色分辨率 | 640×640 @ 30fps |
| 深度分辨率 | 640×640 @ 30fps |
| IMU | 内置 IMU (用于点云去噪等) |
| 深度范围 | 0.2m ~ 10m |

### 1.3 计算平台

| 属性 | 值 |
|------|-----|
| 操作系统 | Ubuntu 22.04 |
| ROS 版本 | ROS 2 Humble |
| 控制主机 | 桌面工作站 |
| 存储路径 | `/media/hcp/disk/data/` |
| 数据盘容量 | 较大（用于存储数据集） |

---

## 二、软件环境

### 2.1 Conda 环境

| 环境名 | 用途 |
|--------|------|
| `gello-env` | dual_camera_collect 主环境 |
| `tidybot2_Franka` | tavp_data_collect 环境 |
| `datacollect` | franka-gello 环境 |

### 2.2 主要依赖

#### Python 包

```
rclpy              # ROS 2 Python 客户端
sensor_msgs        # ROS 消息类型
pyorbbecsdk        # Orbbec 相机 SDK
pyrealsense2       # RealSense 相机 SDK
opencv-python      # 图像处理
numpy              # 数值计算
pyarrow            # Parquet 支持
imageio            # 视频写入
scipy              # 四元数转欧拉角
```

#### ROS 2 包

```
franka_ros2                # Fr3 机械臂 ROS 2 驱动
franka_gello               # Gello 遥操作框架
franka_gripper             # 夹爪控制
franka_robot_state_broadcaster  # 状态发布
realsense2_camera          # RealSense ROS 驱动
```

### 2.3 外参文件

| 文件 | 说明 |
|------|------|
| `T_base_orbbec_1119.npy` | Orbbec 相机外参（基座到相机坐标系） |
| 格式 | 4×4 numpy 矩阵 |
| 标定日期 | 11月19日 |

---

## 三、网络架构

```
┌─────────────────────────────────────────────────────────┐
│                    工作站 (桌面 PC)                      │
│                                                          │
│   ┌─────────────────────────────────────────────────┐   │
│   │              ROS 2 Humble                        │   │
│   │                                                  │   │
│   │   /franka_robot_state_broadcaster/current_pose  │   │
│   │   /franka_gripper/joint_states                  │   │
│   │   /gello/joint_states                           │   │
│   └─────────────────────────────────────────────────┘   │
│                    │                                      │
│                    │ IP: 172.16.0.x                       │
└────────────────────┼────────────────────────────────────┘
                     │
     ┌───────────────┼───────────────┐
     │               │               │
     ▼               ▼               ▼
┌─────────┐   ┌─────────────┐   ┌──────────┐
│ Fr3     │   │ Orbbec Astra│   │RealSense │
│ 机械臂  │   │ (前置) USB  │   │ D435i    │
│         │   │             │   │(腕部)USB │
│172.16.0.2│   │             │   │          │
└─────────┘   └─────────────┘   └──────────┘
```

---

## 四、数据存储结构

### 4.1 磁盘目录

```
/media/hcp/disk/data/
├── tavp_dataset/           # franka-gello 默认存储
├── tavp_dataset_extra/     # dual_camera_collect 存储
└── unscrew_cap/            # tavp_data_collect 存储
```

### 4.2 录制数据格式

#### LeRobot v2.1 格式 (dual_camera_collect)

```
{BASE_DIR}/
└── {task_name}/
    └── data_001/
        ├── videos/
        │   ├── observation.images.third_person.mp4
        │   └── observation.images.wrist.mp4
        ├── frontImg/000000.png ~ 999999.png
        ├── frontDep/000000.npy ~ 999999.npy
        ├── frontPcd/000000.npy ~ 999999.npy
        ├── wristImg/
        ├── wristDep/
        ├── wristPcd/
        └── data.parquet
```

#### Parquet Schema

| 字段 | 类型 | 说明 |
|------|------|------|
| observation.state | list[float64] | 11维: [x,y,z,roll,pitch,yaw,qx,qy,qz,qw,gripper] |
| observation.gello_joints | list[float64] | 7维关节角度 |
| action | list[float64] | 与 state 相同 |
| timestamp | float64 | Unix 时间戳 |
| gripper_width | float64 | 夹爪间距 (m) |

---

## 五、启动流程

### 5.1 dual_camera_collect 启动顺序

```bash
# 1. 激活 conda 环境
conda activate gello-env

# 2. 加载 ROS 2 环境
source /opt/ros/humble/setup.bash
source ~/workspace/franka_ros2_ws/install/setup.bash

# 3. 加载 Gello 软件环境
cd ~/gello_software/ros2/
source install/setup.bash

# 4. 启动 Gello 系统节点 (后台)
cd $HOME/gello_software/ros2/
source install/setup.bash
ros2 launch franka_gellobringup T1_State_Publisher.py &
# ... 其他节点

# 5. 启动数据采集程序
cd /home/hcp/workspace/tavp_proj/tavp_collect_data_gello/franka-data-collect/dual_camera_collect
python dual_camera_collector.py --task_name fold_towel
```

### 5.2 启动脚本

| 脚本 | 用途 |
|------|------|
| `start_gello.sh` | 启动 Gello 系统节点 |
| `start_dual_camera.sh` | 一键启动双相机采集 |
| `start_data_collect.sh` | 启动数据采集程序 |

---

## 六、键盘控制

| 按键 | 功能 |
|------|------|
| `s` | 开始录制 |
| `e` | 停止录制 |
| `q` | 退出程序 |

---

## 七、故障排查

### 7.1 相机无法初始化

```bash
# 检查 USB 连接
lsusb | grep -E "Orbbec|RealSense"

# 检查相机权限
sudo chmod +666 /dev/bus/usb/*
```

### 7.2 ROS topic 订阅不到数据

```bash
# 检查 topic 列表
ros2 topic list

# 查看特定 topic
ros2 topic echo /franka_robot_state_broadcaster/current_pose
```

### 7.3 机械臂连接失败

```bash
# ping 测试
ping 172.16.0.2

# recover
python -c "from franky import Robot; r = Robot('172.16.0.2'); r.recover_from_errors()"
```

---

## 八、环境版本记录

| 日期 | 环境 | 备注 |
|------|------|------|
| 2024-11 | 初始 setup | - |
| 2024-11 | Orbbec 外参标定 | T_base_orbbec_1119.npy |
| 2025-05 | dual_camera_collect 上线 | 多线程架构 |
| 2025-05 | tavp_data_collect lesson | 轨迹示教重放 |
| 2025-05 | franka-gello lesson | 单相机采集 |

---

## 九、常见问题速查

| 问题 | 可能原因 | 解决方案 |
|------|---------|----------|
| Orbbec 无图像 | USB 带宽不足 / 4K 模式 | 关闭深度流或降低分辨率 |
| RealSense 无图像 | USB 供电不足 | 使用带供电的 USB hub |
| 机械臂无法连接 | IP 不通 / 防护模式 | ping 测试 + recover |
| 数据保存失败 | 磁盘满 / 权限问题 | 检查 `/media/hcp/disk/` 空间 |
| 录制状态卡住 | 写入阻塞主线程 | 检查是否使用了独立写入线程 |