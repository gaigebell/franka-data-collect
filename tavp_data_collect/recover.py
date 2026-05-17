import time
from franky import *
import math
import numpy as np
import time

# Move to a safe joint configuration first (avoiding singularities)
def recover_robot(robot_ip):
    robot = Robot(robot_ip)
    gripper = Gripper(robot_ip)
    robot.relative_dynamics_factor = 0.1
    safe_joints = [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4]
    safe_motion = JointMotion(safe_joints, ReferenceType.Absolute)
    robot.move(safe_motion)
    time.sleep(0.5)  # Wait for the robot to reach the position
    gripper.open(speed=0.02)
    print("safe position reached.")
if __name__ == "__main__":
    recover_robot("172.16.0.2")
