conda activate gello-env
source /opt/ros/humble/setup.bash
# source ~/franka_ros2_ws/install/setup.bash
source ~/workspace/franka_ros2_ws/install/setup.bash
cd /home/hcp/gello_software/ros2/
source install/setup.bash

#T0
ros2 launch franka_gello_state_publisher main.launch.py config_file:=single.yaml

#T1
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:=fr3_config.yaml

#T2
ros2 launch franka_gripper_manager franka_gripper_client.launch.py config_file:=fr3_hand_yjh.yaml

#T3: 启动相机
ros2 launch realsense2_camera rs_launch.py


