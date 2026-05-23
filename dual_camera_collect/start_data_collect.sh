source $HOME/miniforge3/bin/activate tidybot2_Franka
source /opt/ros/humble/setup.bash
# source ~/franka_ros2_ws/install/setup.bash
source ~/workspace/franka_ros2_ws/install/setup.bash
cd /home/hcp/gello_software/ros2/
source install/setup.bash
cd /home/hcp/workspace/tavp_proj/tavp_collect_data_gello/franka-data-collect/dual_camera_collect
#  /home/hcp/workspace/tavp_proj/tavp_collect_data_gello/franka-data-collect/dual_camera_collect
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
    --task_name fold_towel \
    --base_dir /media/hcp/disk/data/tavp_dataset_extra

