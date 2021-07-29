#!/usr/bin/env python3

import rospy
import math
from commander_api.motion_control_client import MotionControlClient, Waypoint, Motion

rospy.init_node("motion_advanced")

mc = MotionControlClient("default_move_group")

ACCEL = 0.4
VEL = 0.2
BLENDL = 0.1

m = Motion()

for x in range(50):
    m.add(Waypoint(0.4, x/50 - 0.5, 0.4, math.pi, 0, 0) \
        .constrain_joint_acceleration(ACCEL) \
        .constrain_joint_velocity(VEL) \
        .set_blend(BLENDL, 0.5) \
        .set_linear())
    m.add(Waypoint(0.4, x/50 - 0.5, 0.41, math.pi, 0, 0) \
        .constrain_joint_acceleration(ACCEL) \
        .constrain_joint_velocity(VEL) \
        .set_blend(BLENDL, 0.5) \
        .set_linear())
    m.add(Waypoint(0.4, 0.01 + x/50 - 0.5, 0.41, math.pi, 0, 0) \
        .constrain_joint_acceleration(ACCEL) \
        .constrain_joint_velocity(VEL) \
        .set_blend(BLENDL, 0.5) \
        .set_linear())
    m.add(Waypoint(0.4, 0.01 + x/50 - 0.5, 0.41, math.pi, 0, 0) \
        .constrain_joint_acceleration(ACCEL) \
        .constrain_joint_velocity(VEL) \
        .set_blend(BLENDL, 0.5) \
        .set_linear())
mc.run(m)

print("Done!")
