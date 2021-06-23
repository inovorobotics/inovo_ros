#!/usr/bin/env python3
import rospy
import actionlib  # SimpleActionClient
from arm_msgs.msg import *
from arm_msgs.srv import *
from commander_msgs.msg import *  # MotionAction and friends
from lease.srv import *
from lease.msg import *
from commander_api.custom_datatypes import TransformFrame
from geometry_msgs.msg import *
import threading

import PyKDL


class Waypoint:
    """
    A waypoint describes a desired robot pose and how the robot should get there
    """

    def __init__(self, x, y, z, rx, ry, rz):
        self._point = MotionSequencePoint()
        self._point.pose.position.x = x
        self._point.pose.position.y = y
        self._point.pose.position.z = z

        rot = PyKDL.Rotation.EulerZYX(rz, ry, rx)
        quat = rot.GetQuaternion()
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
    Container for a series of motions.
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
    """Class used to send motion commands to the robot
    """

    def __init__(self, ns="default_move_group"):
        self.client = actionlib.SimpleActionClient(ns + "/move", MotionAction)
        if not self.client.wait_for_server(rospy.Duration(1.0)):
            raise Exception("Unable to connect to action server")

    def movej_angle(self, joint_angles, joint_velocity=0.4, joint_acceleration=0.75):
        """Move to a joint space target, locks till the target is reached

        :param joint_angles: Joint angles in radians
        :type joint_angles: list(float)
        :param joint_velocity: Maximum joint velocity in rad/s, defaults to 0.4
        :type joint_velocity: float, optional
        :param joint_acceleration: Maximum joint acceleration in rad/s/s, defaults to 0.75
        :type joint_acceleration: float, optional
        :raises RuntimeError: Unable to execute the move
        """
        point = MotionSequencePoint()
        point.max_joint_velocity = joint_velocity
        point.max_joint_acceleration = joint_acceleration
        point.use_joint_space_target = True
        point.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        point.joint_angles = joint_angles
        point.unwind_joint_target = True

        # ping from thread lambda callback timer while flag to get out
        try:
            self.acquire_lease(self.unique_id, self.hardware_id)
        except:
            print('Coulnt acquire lease')

        self.lease_flag = True

        t1 = threading.Thread(target=self.__lease_thread)
        t1.start()

        # Add the waypoint to the sequence
        goal = MotionGoal()
        goal.motion_sequence.append(point)

        self.client.send_goal(goal)
        self.client.wait_for_result()
        res = self.client.get_result()

        # Release the lease
        self.lease_flag = False
        t1.join()

        if not res.success:
            raise RuntimeError("Unable to run sequence: " + res.message)

    def run(self, motion):
        goal = MotionGoal()
        goal.motion_sequence = motion.get_sequence()

        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        if not result.success:
            raise Exception(result.message)

    def movej(self, target, spd=0.25, acc=0.5):
        """Joint move to a cartesian target, locks till the target is reached

        :param target: Target position x, y, z in meters and rotation rx, ry, rz in radians
        :type target: TransformFrame
        :param spd: Tool speed in m/s, defaults to 0.25
        :type spd: float, optional
        :param acc: Tool acceleration in m/s/s, defaults to 0.5
        :type acc: float, optional
        :raises RuntimeError: Unable to execute the move
        """
        point = MotionSequencePoint()
        point.pose.position.x = target.x
        point.pose.position.y = target.y
        point.pose.position.z = target.z

        quat = target.get_quaternion()

        point.pose.orientation.x = quat[0]
        point.pose.orientation.y = quat[1]
        point.pose.orientation.z = quat[2]
        point.pose.orientation.w = quat[3]

        point.max_velocity.linear = spd
        point.max_acceleration.linear = acc
        point.cartesian = False

        # ping from thread lambda callback timer while flag to get out
        self.acquire_lease(self.unique_id, self.hardware_id)
        self.lease_flag = True
        t1 = threading.Thread(target=self.__lease_thread)
        t1.start()

        # Add the waypoint to the sequence
        goal = MotionGoal()
        goal.motion_sequence.append(point)

        self.client.send_goal(goal)
        self.client.wait_for_result()
        res = self.client.get_result()

        # Release the lease
        self.lease_flag = False
        t1.join()

        if not res.success:
            raise RuntimeError("Unable to run sequence: " + res.message)

    def movel(self, target, spd=0.25, acc=0.5):
        """Linear move to a cartesian target, locks till the target is reached

        :param target: Target position x, y, z in meters and rotation rx, ry, rz in radians
        :type target: TransformFrame
        :param spd: Tool speed in m/s, defaults to 0.25
        :type spd: float, optional
        :param acc: Tool acceleration in m/s/s, defaults to 0.5
        :type acc: float, optional
        :raises RuntimeError: Unable to execute the move
        """

        point = MotionSequencePoint()
        point.pose.position.x = target.x
        point.pose.position.y = target.y
        point.pose.position.z = target.z

        quat = target.get_quaternion()

        point.pose.orientation.x = quat[0]
        point.pose.orientation.y = quat[1]
        point.pose.orientation.z = quat[2]
        point.pose.orientation.w = quat[3]

        point.max_velocity.linear = spd
        point.max_acceleration.linear = acc
        point.cartesian = True

        print('starting lease')
        # ping from thread lambda callback timer while flag to get out
        self.acquire_lease(self.unique_id, self.hardware_id)
        self.lease_flag = True
        print('starting thread')
        t1 = threading.Thread(target=self.__lease_thread)
        t1.start()

        # Add the waypoint to the sequence
        goal = MotionGoal()
        goal.motion_sequence.append(point)

        self.client.send_goal(goal)
        self.client.wait_for_result()
        res = self.client.get_result()

        # Release the lease
        self.lease_flag = False
        t1.join()

        if not res.success:
            raise RuntimeError("Unable to run sequence: " + res.message)

    def get_joint_angles(self):
        """Get the current joint positions

        :return: Joint positions (rad)
        :rtype: list(float)
        """
        res = self.get_joint_states_srv()
        return res.state.position

    def get_tcp_pose(self):
        """Get the current position of the TCP in base frame 

        :return: TCP position
        :rtype: TransformFrame
        """
        tcp = rospy.wait_for_message(
            "/default_move_group/tcp_pose", TransformStamped)
        frame = TransformFrame()

        frame.x = tcp.transform.translation.x
        frame.y = tcp.transform.translation.y
        frame.z = tcp.transform.translation.z
        frame.from_quaternion(tcp.transform.rotation.x, tcp.transform.rotation.y,
                              tcp.transform.rotation.z, tcp.transform.rotation.w)
        return frame
