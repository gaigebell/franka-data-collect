#!/bin/bash
echo "🛑 正在停止 Gello 系统节点..."

# 方法 A: 杀死由启动脚本记录的 PID (如果你保留了终端)
# 如果没有保留 PID 变量，使用方法 B 更通用

# 方法 B: 通过进程名杀死 (推荐)
# 注意：这会杀死当前用户下所有相关的 ros2 launch 进程
pkill -f "franka_gello_state_publisher"
pkill -f "franka_fr3_arm_controllers"
pkill -f "franka_gripper_manager"
pkill -f "realsense2_camera"

# 也可以尝试停止整个 ros2 系统 (慎用，会停止所有 ros2 节点)
# ros2 lifecycle set $(ros2 node list | head -n1) shutdown 

echo "✅ 发送了停止信号。如果节点未立即消失，请稍等或使用 'kill -9'。"