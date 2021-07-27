#!/usr/bin/env python3
import rospy
import math
from commander_api.robot_client import RobotClient
import time

rospy.init_node('robot_example')
robot = RobotClient('/robot')

robot.disable()

time.sleep(0.5)

robot.enable()

robot.switch_controller("trajectory")
