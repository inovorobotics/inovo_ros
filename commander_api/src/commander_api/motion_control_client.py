#!/usr/bin/env python3
import rospy
import actionlib  # SimpleActionClient
from commander_msgs.msg import *  # MotionAction and friends
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf

class Waypoint:
    """
    A waypoint describes a desired robot pose and how the robot should get there
    """

    def __init__(self, x, y, z, rx, ry, rz):
        self._point = MotionSequencePoint()
        self._point.pose.position.x = x
        self._point.pose.position.y = y
        self._point.pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(rx, ry, rz)
        self._point.pose.orientation.x = quat[0]
        self._point.pose.orientation.y = quat[1]
        self._point.pose.orientation.z = quat[2]
        self._point.pose.orientation.w = quat[3]

    def set_linear(self):
        self._point.cartesian = True
        return self

    def constrain_joint_velocity(self, velocity):
        self._point.max_joint_velocity = velocity
        return self

    def constrain_joint_acceleration(self, acceleration):
        self._point.max_joint_acceleration = acceleration
        return self

    def constrain_tcp_velocity(self, linear, angular):
        self._point.max_velocity.linear = linear
        self._point.max_velocity.angular = angular
        return self

    def set_blend(self, linear, angular):
        self._point.blend.linear = linear
        self._point.blend.angular = angular
        return self

    def get_point(self):
        return self._point


class Motion:
    """
    Container for a set of motions.
    """

    def __init__(self, point=None):
        self.points = []
        if point is not None:
            self.add(point)

    def get_sequence(self):
        ret = []
        for p in self.points:
            ret.append(p.get_point())
        return ret

    def add(self, point):
        try:
            self.points.extend(point)
        except TypeError:
            self.points.append(point)


class MotionControlClient:
    """
    Client used to send high level motion commands to an Inovo robot.
    """

    def __init__(self, ns="default_move_group"):
        self.ns = ns
        self.client = actionlib.SimpleActionClient(ns + "/move", MotionAction)
        if not self.client.wait_for_server(rospy.Duration(1.0)):
            raise Exception("Unable to connect to action server")

    def run(self, motion):
        goal = MotionGoal()
        goal.motion_sequence = motion.get_sequence()

        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        if not result.success:
            raise Exception(result.message)

    def get_joint_angles(self):
        """Get the current joint positions

        :return: Joint positions (rad)
        :rtype: list(float)
        """
        msg = rospy.wait_for_message("/joint_states", JointState, rospy.Duration(1.0))
        return msg

    def get_tcp_pose(self):
        """Get the current position of the TCP in base frame 

        :return: TCP position
        :rtype: geometry_msgs.msg.Pose
        """
        msg = rospy.wait_for_message(self.ns + "/tcp_pose", PoseStamped, rospy.Duration(1.0))
        return msg.pose
