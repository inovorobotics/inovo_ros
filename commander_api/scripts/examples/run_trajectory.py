#!/usr/bin/env python3
import rospy
import time
import actionlib
from commander_api.robot_client import RobotClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('robot_example')

# The robot client is used to switch the controller
robot = RobotClient('/robot')
trajectory_action_client = actionlib.SimpleActionClient('/robot/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

if not trajectory_action_client.wait_for_server(rospy.Duration(1.0)):
    raise Exception("No action server!")

# Build up a trajectory which goes to all 0's at t=3 and all 1's at t=6
trajectory = JointTrajectory()

point = JointTrajectoryPoint()
point.positions = [0.0] * 6
point.velocities = [0.0] * 6
point.accelerations = [0.0] * 6
point.time_from_start = rospy.Duration(3.0)
trajectory.points.append(point)

point = JointTrajectoryPoint()
point.positions = [1.0] * 6
point.velocities = [0.0] * 6
point.accelerations = [0.0] * 6
point.time_from_start = rospy.Duration(6.0)
trajectory.points.append(point)

goal = FollowJointTrajectoryGoal()
goal.trajectory = trajectory

# The robot must be switched into the "trajectory" controller before trajectories can be executed
robot.switch_controller("trajectory")

# Send the goal and wait for it to finish
# WARNING: This command will move the robot
trajectory_action_client.send_goal_and_wait(goal);
