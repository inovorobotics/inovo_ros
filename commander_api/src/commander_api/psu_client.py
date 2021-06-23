#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger
from gpio_msgs.srv import *
from psud.msg import *
from psud.srv import BusControl

class PSUClient:
    """Class used to control the PSU box
    """    
    def __init__(self):
        self.shut_down_srv = rospy.ServiceProxy('/poweroff', Trigger)
        self.bus_control_srv = rospy.ServiceProxy('/psu/bus_control', BusControl)
        self.status_subscriber = rospy.Subscriber('/psu/status',Status, self.__status_callback)
        self.state = ""
        self.bus_current = 0.0
        self.bus_current = 0.0

    def __status_callback(self,msg):
        self.state = msg.state
        self.bus_voltage = msg.voltage
        self.bus_current = msg.current


    def shut_down(self):
        """Turn off the PSU
        """        
        self.shut_down_srv()

    def power_on(self):
        """Robot bus power on

        :raises Exception: Error powering on the robot
        """           
        res=self.bus_control_srv(True)
        if res.success == False:
            raise Exception("Error powering on the robot")


    def power_off(self):
        """Robot bus power off

        :raises Exception: Error powering off the robot
        """        
        res = self.bus_control_srv(False)
        if res.success == False:
            raise Exception("Error powering off the robot")

    def get_bus_state(self):
        """Get the bus power status

        :raises Exception: Unknown psu state
        :return: State, bus on = 1, bus off = 0
        :rtype: int
        """        
        if self.state == "BUS_ON":
            return 1
        elif self.state == "BUS_OFF":
            return 0
        else:
            raise Exception("Uknown psu state") 

    def get_bus_voltage(self):
        """Get the voltage of the bus

        :return: Bus voltage
        :rtype: float
        """        
        return self.bus_voltage

    def get_bus_current(self):
        """Get the current of the bus

        :return: Bus current
        :rtype: float
        """        
        return self.bus_current