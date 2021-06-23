#!/usr/bin/env python3
import rospy
import actionlib # SimpleActionClient
from control_msgs.srv import *
from control_msgs.msg import *
from std_srvs.srv import Trigger
from gripper_msgs.msg import GripperState

class GripperClient:
    """Class to controll gripper connected to the robot

        :param ns: Namespace, to define the name of the gripper, defaults to 'default'
        :type ns: string, optional
    """
    def __init__(self, ns = 'default'):
        
        self.client = actionlib.SimpleActionClient("/grippers/{}/gripper_command".format(ns), GripperCommandAction)
        self.client.wait_for_server(rospy.Duration(1))
        self.activate_srv = rospy.ServiceProxy('/grippers/{}}/activate'.format(ns), Trigger)
        self.ns = ns
            

    def activate(self):
        """Activate the gripper before moving it

        :raises Exception: Gripper activation error
        """              
        res = self.activate_srv()
        if res.success == False:
            raise Exception("Gripper activation error")


    def move(self, target_position, effort):
        """Move the gripper to the given goal, locks till target is reached or gripper is stalled

        :param target_position: The desired position of the gripper, from 0 to 1
        :type target_position: float
        :param effort: The effort used to move the fingers, from 0 to 1
        :type effort: float
        :raises Exception: Gripper action error
        :return: True if the target position is not reached, False if the target is reached
        :rtype: Bool
        """ 
        goal = GripperCommand()
        goal.position = target_position
        goal.max_effort = effort

        self.client.send_goal(goal)
        self.client.wait_for_result() # optional?
        res = self.client.get_result()
        if res.status != 3:
            raise Exception("Gripper action error") 
        #TODo
        if self.gripper_postion == target_position:
            return False
        else:
            return True

        

    def get_position(self):
        """Return current position of the gripper

        :return: Gripper position
        :rtype: float
        """ 
        msg = rospy.wait_for_message('/grippers/{}/gripper_state'.format(self.ns),GripperState)                 
        return msg.gripper_postion
    
    def get_state(self):
        """Return information about activation state of the gripper
        

        :return: Gripper state
        :rtype: bool
        """    
        msg = rospy.wait_for_message('/grippers/{}/gripper_state'.format(self.ns),GripperState)  
        return msg.gripper_state      