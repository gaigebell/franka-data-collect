#!/bin/bash

# ============================================================================
# 双相机数据采集系统启动脚本
# ============================================================================

# 4. 切换到 dual_camera_collect 目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "------------------------------------------"
echo "📡 正在启动双相机数据采集..."
echo "------------------------------------------"

# 启动采集程序
python dual_camera_collector.py \
    --enable_orbbec true \
    --enable_realsense true \
    --orbbec_color true \
    --orbbec_depth false \
    --realsense_color true \
    --realsense_depth false \
    --save_video true \
    --save_images true \
    --fps 30 \
    --task_name dual_test \
    --base_dir /media/hcp/disk/data/tavp_dataset_extra