#!/usr/bin/env python3
import rospy
import math
from commander_api.gripper_client import GripperClient

pi = math.pi

rospy.init_node('example')
gripper = GripperClient()


gripper.activate()

#open gripper with half force
gripper.set_goal(1.0 , 0.5)

#close gripper with half force
gripper.set_goal(0.0 , 0.5)

