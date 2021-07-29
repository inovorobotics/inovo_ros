#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger
from inovo_driver_msgs.srv import SwitchControllerGroup

class RobotClient:
    """Robot client class used for basic control of the robot
    """    
    def __init__(self, ns='/robot'):
        self.disable_srv = rospy.ServiceProxy(ns + '/disable', Trigger)
        self.enable_srv = rospy.ServiceProxy(ns + '/enable', Trigger)
        self.switch_controller_srv = rospy.ServiceProxy(ns + '/switch_controller', SwitchControllerGroup)
    
    def enable(self):
        """Enable the robot when it is powered on

        :raises Exception: Failed to enable the robot
        """             
        res = self.enable_srv()
        if res.success == False:
            raise Exception("Failed to enable the robot")

    def disable(self):
        """Disable the robot

        :raises Exception: Failed to disable the robot
        """     
        res = self.disable_srv()
        if res.success == False:
            raise Exception("Failed to disable the robot")
    
    def switch_controller(self, controller_type):
        """
        Switch which controller is running.

        :param controller_type: The name of the controller to switch to. Possible values: "trajectory", "velocity", "zerog"
        """
        res = self.switch_controller_srv(controller_type)
        if res.success == False:
            raise Exception(f"Failed to switch controller to {controller_type}")
