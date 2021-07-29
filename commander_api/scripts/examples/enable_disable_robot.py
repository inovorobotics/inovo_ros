#!/usr/bin/env python3
import rospy
import time
from commander_api.robot_client import RobotClient

rospy.init_node('robot_example')
robot = RobotClient('/robot')

robot.disable()
time.sleep(0.5)
robot.enable()
