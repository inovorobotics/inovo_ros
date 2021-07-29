#!/usr/bin/env python3
import rospy
from commander_api.motion_control_client import MotionControlClient

rospy.init_node("get_robot_state")

mc = MotionControlClient("default_move_group")

print("Robot's TCP pose :", mc.get_tcp_pose())
print("Robot's joint angles:", mc.get_joint_angles())
