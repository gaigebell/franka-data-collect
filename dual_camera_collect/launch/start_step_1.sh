#!/bin/bash

# ============================================================================
# 双相机数据采集系统启动脚本
# ============================================================================

echo "🚀 启动双相机数据采集系统..."

# 1. 激活 Conda 环境
if command -v conda &> /dev/null; then
    eval "$(conda shell.bash hook)"
    conda activate gello-env
    echo "✅ Conda 环境 'gello-env' 已激活"
else
    echo "❌ 错误: 未找到 conda 命令"
    exit 1
fi

# 2. 加载 ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS 2 Humble 环境已加载"
else
    echo "❌ 错误: 未找到 /opt/ros/humble/setup.bash"
    exit 1
fi

# 3. 加载 Franka ROS 2 工作空间
if [ -f ~/workspace/franka_ros2_ws/install/setup.bash ]; then
    source ~/workspace/franka_ros2_ws/install/setup.bash
    echo "✅ Franka ROS 2 工作空间已加载"
else
    echo "❌ 错误: 未找到 ~/workspace/franka_ros2_ws/install/setup.bash"
    exit 1
fi