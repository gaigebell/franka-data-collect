#!/bin/bash

# ==========================================
# Gello Software System Startup Script
# ==========================================

echo "🚀 正在初始化环境..."

# 1. 激活 Conda 环境
# 注意：如果 conda 不在 PATH 中，可能需要 source ~/anaconda3/etc/profile.d/conda.sh
if command -v conda &> /dev/null; then
    eval "$(conda shell.bash hook)"
    conda activate gello-env
    echo "✅ Conda 环境 'gello-env' 已激活"
else
    echo "❌ 错误: 未找到 conda 命令，请检查安装或 PATH。"
    exit 1
fi

# 2. 加载 ROS 2 Humble 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS 2 Humble 环境已加载"
else
    echo "❌ 错误: 未找到 /opt/ros/humble/setup.bash"
    exit 1
fi

# 3. 加载 Franka ROS 2 工作空间
# if [ -f ~/franka_ros2_ws/install/setup.bash ]; then
#     source ~/franka_ros2_ws/install/setup.bash
#     echo "✅ franka_ros2_ws 工作空间已加载"
# else
#     echo "❌ 错误: 未找到 ~/franka_ros2_ws/install/setup.bash"
#     exit 1
# fi

# ~/workspace/franka_ros2_ws/install/setup.bash
if [ -f ~/workspace/franka_ros2_ws/install/setup.bash ]; then
    source ~/workspace/franka_ros2_ws/install/setup.bash
    echo "✅ franka_ros2_ws 工作空间已加载"
else
    echo "❌ 错误: 未找到 ~/workspace/franka_ros2_ws/install/setup.bash"
    exit 1
fi


# 4. 切换目录并加载 Gello 软件环境
TARGET_DIR="/home/hcp/gello_software/ros2/"
if [ -d "$TARGET_DIR" ]; then
    cd "$TARGET_DIR"
    if [ -f install/setup.bash ]; then
        source install/setup.bash
        echo "✅ Gello ROS 2 局部环境已加载 (位于 $TARGET_DIR)"
    else
        echo "⚠️ 警告: 未找到 $TARGET_DIR/install/setup.bash，继续执行..."
    fi
else
    echo "❌ 错误: 目录 $TARGET_DIR 不存在"
    exit 1
fi

echo "------------------------------------------"
echo "📡 正在启动 ROS 2 节点..."
echo "------------------------------------------"

# 定义日志目录 (可选，方便调试)
LOG_DIR="$HOME/hcp_logs/gello_logs_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

# 启动函数：在后台运行并记录日志
# 参数 1: 任务名称, 参数 2: 启动命令
launch_node() {
    local name=$1
    shift
    echo "🔹 启动 [$name] ..."
    # 使用 ros2 launch ... & 将其放入后台
    # 输出重定向到日志文件
    ros2 launch "$@" > "$LOG_DIR/$name.log" 2>&1 &
    PIDS+=($!) # 保存进程ID
    echo "   -> PID: ${PIDS[-1]}, 日志: $LOG_DIR/$name.log"
}

# 数组用于存储后台进程 ID
PIDS=()

# T0: 状态发布器
launch_node "T0_State_Publisher" franka_gello_state_publisher main.launch.py config_file:=single.yaml

# 短暂延迟，确保参数服务器就绪 (根据实际需求调整秒数)
sleep 2

# T1: 机械臂控制器
launch_node "T1_Arm_Controllers" franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:=fr3_config.yaml

# T2: 夹爪管理器
launch_node "T2_Gripper_Manager" franka_gripper_manager franka_gripper_client.launch.py config_file:=fr3_hand_yjh.yaml

# T3: 相机 (RealSense)
launch_node "T3_Camera" realsense2_camera rs_launch.py depth_module.color_profile:=640x480x60 enable_depth:=false enable_infra:=false


echo "------------------------------------------"
echo "✨ 所有节点已启动!"
echo "📂 日志文件保存在: $LOG_DIR"
echo "💡 提示: 按 Ctrl+C 将停止当前脚本，但后台节点可能仍在运行。"
echo "   若要停止所有相关节点，请运行: kill ${PIDS[*]}"
echo "   或者直接使用下面的停止脚本。"
echo "------------------------------------------"

# 等待所有后台进程结束 (如果你希望脚本挂起直到所有节点停止)
# 如果希望脚本执行完就退出而让节点在后台跑，注释掉下面这行
wait "${PIDS[@]}"