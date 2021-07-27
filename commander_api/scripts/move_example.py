#!/usr/bin/env python3
import rospy
import math
from commander_api.motion_control_client import MotionControlClient, Waypoint, Motion
from commander_api.custom_datatypes import TransformFrame
from commander_msgs.msg import MotionAction, MotionGoal, MotionSequencePoint

pi = math.pi

# Initialize the ROS node
rospy.init_node("commander_api_example")

# Create a new motion control client
mc = MotionControlClient("default_move_group")

a = Waypoint(0.4, 0.4, 0.4, math.pi, 0, 0)
b = Waypoint(0.4, -0.4, 0.4, math.pi, 0, 0)

print("Moving to a")
mc.run(Motion(a))

print("Moving to b")
mc.run(Motion(b))

print("Moving to a then b in one motion")
mc.run(Motion((a, b)))

print("Running a complex path")
m = Motion()

ACCEL = 1.0
VEL = 1.0
BLENDL = 0.2

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
