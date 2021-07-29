#!/usr/bin/env python3

import rospy
import math
from commander_api.motion_control_client import MotionControlClient, Waypoint, Motion

rospy.init_node("motion_simple")

mc = MotionControlClient("default_move_group")

mc.run(Motion(Waypoint(0.55, -0.25, 0.5, math.pi, 0, 0)))
mc.run(Motion(Waypoint(0.55, -0.15, 0.5, math.pi, 0, 0)))

print("Done!")
