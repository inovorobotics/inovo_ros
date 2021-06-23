#!/usr/bin/env python3
import rospy
from arm_msgs.msg import *
from std_msgs import *
from std_srvs.srv import Trigger
from error_logger.srv import GetErrors
from error_msgs.msg import *
from commander_api.custom_datatypes import *

import math
import time

pi = math.pi

class RobotClient:
    """Robot client class used for basic controll of the robot functionality
    """    
    def __init__(self):
        self.disable_srv = rospy.ServiceProxy('/robot/disable', Trigger)
        self.enable_srv = rospy.ServiceProxy('/robot/enable', Trigger)
        self.get_errors_srv = rospy.ServiceProxy('/get_errors', GetErrors)
        self.clear_errors_srv = rospy.ServiceProxy('/clear_errors', Trigger)
        
    
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



    def get_errors(self):
        """Print the latest error message

        :return: Error message
        :rtype: ErrorMessage
        """    
        res = self.get_errors_srv()
        error = ErrorMessage()
        if len(res.errors) > 0:
            error.message = res.errors[-1].message
            error.error_code = res.errors[-1].error_code
            error.source = res.errors[-1].source

        return error
        

    def clear_errors(self):
        """Clear all errors 

        :raises Exception: Failed to clear errors
        """           
        res = self.clear_errors_srv()
        if res.success == False:
            raise Exception("Failed to clear errors")
