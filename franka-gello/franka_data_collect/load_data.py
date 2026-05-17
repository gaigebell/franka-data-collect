import pandas as pd
import matplotlib.pyplot as plt

# 读取你的 parquet 文件
df = pd.read_parquet("path/to/your/data.parquet")

# 查看同步差值的统计信息
print(df[['sync_diff_pose_ms', 'sync_diff_grip_ms']].describe())

# 绘制直方图
plt.figure(figsize=(10, 5))
plt.hist(df['sync_diff_pose_ms'], bins=50, alpha=0.6, label='Pose Diff (ms)')
plt.hist(df['sync_diff_grip_ms'], bins=50, alpha=0.6, label='Gripper Diff (ms)')
plt.axvline(0, color='red', linestyle='--', label='Perfect Sync')
plt.xlabel('Time Difference (ms) relative to Image')
plt.ylabel('Count')
plt.legend()
plt.title('Sensor Synchronization Analysis')
plt.show()

# 测试读取一张图片
first_img = df['image_rgb'].iloc[0]
print(f"Image shape loaded directly: {first_img.shape}") # 应该输出 (H, W, 3)