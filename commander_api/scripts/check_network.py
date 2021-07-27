#!/usr/bin/env python3

# A simple script that attempts to connect to the "move" action server on an Inovo robot and reports whether it could connect or not.

import rospy
import actionlib
from commander_msgs.msg import MotionAction

rospy.init_node('check_network')

client = actionlib.SimpleActionClient('/default_move_group/move', MotionAction)

if client.wait_for_server():
    print("Connected to the 'move' action server. Network is configured correctly!")
else:
    print("Timed out waiting for server to connect. Your network is not configured properly.")
