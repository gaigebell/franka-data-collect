source $HOME/miniforge3/bin/activate datacollect
source /opt/ros/humble/setup.bash
# source ~/franka_ros2_ws/install/setup.bash
source ~/workspace/franka_ros2_ws/install/setup.bash
cd /home/hcp/gello_software/ros2/
source install/setup.bash
cd /media/hcp/disk/projects/franka_data_collect
python data_collect.py --task_name Test
# python data_collect.py --task_name PickUpBanana
# python data_collect.py --task_name PickUpRedApple
# python data_collect.py --task_name PickUpGreenApple
# python data_collect.py --task_name PutRedAppleInBowl
# python data_collect.py --task_name PutBananaInBowl
# python data_collect.py --task_name PickGreenAppleIntoBowl
# python data_collect.py --task_name StackBowlBlueGreenBeige
# python data_collect.py --task_name StackBowlGreenBeigeBlue
# python data_collect.py --task_name PutRedAppleInDrawer
# python data_collect.py --task_name PutGreenAppleIntoDrawer