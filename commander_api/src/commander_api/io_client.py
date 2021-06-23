#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger
from gpio_msgs.srv import *
from psud.srv import *

class IOClient:
    """Class used to control the digital input/output on the robot writst and PSU box
    """    
    def __init__(self):
        self.digital_read_srv = rospy.ServiceProxy('/gpiod/digital_read', DigitalRead)
        self.digital_write_srv = rospy.ServiceProxy('/gpiod/digital_write', DigitalWrite)
        self.wrist_digital_write_srv = rospy.ServiceProxy('/robot/wrist/io/digital_write', DigitalWrite)
        self.wrist_digital_read_srv = rospy.ServiceProxy('/robot/wrist/io/digital_read', DigitalRead)


    def psu_digital_read(self, channel):
        """Get the value of the PSU digital input

        :param channel: Channel to read 
        :type channel: int
        :raises Exception: PSU digital read error
        :return: Value read by the PSU
        :rtype: int
        """
        res = self.digital_read_srv(channel)
        if res.success == False:
            raise Exception("PSU digital read error")
        return res.value


    def psu_digital_write(self, channel, value):
        """Write to the PSU digital output

        :param channel: Channel to write 
        :type channel: int
        :param value: Output value
        :type value: int
        :raises Exception: PSU digital write error
        """  
        res=self.digital_write_srv(channel,value)
        if res.success == False:
            raise Exception("PSU digital write error")

    def psu_analog_read(self, channel):
        """Get the value of the PSU analog input

        :param channel: Channel to read 
        :type channel: int
        :return: Analog input value
        :rtype: float
        """              
        return 0.0

    def psu_analog_write(self, channel, value):
        """Write to the PSU analog output

        :param channel: Channel to write 
        :type channel: int
        :param value: Output value
        :type value: float
        """

    def wrist_digital_read(self, channel):  
        """Get the read of the wrist digital input

        :param channel: Channel to read 
        :type channel: int
        :raises Exception: Wrist digital read error
        :return: Value read by the wrist
        :rtype: int
        """        
        res = self.wrist_digital_read_srv(channel)
        if res.success == False:
            raise Exception("Wrist digital read error")
        return res.value

    def wrist_digital_write(self, channel, value):
        """Write to the wrist digital output

        :param channel: Channel to write 
        :type channel: int
        :param value: Output value
        :type value: int
        :raises Exception: Wrist digital write error
        """        
        res = self.wrist_digital_write_srv(channel,value)
        if res.success == False:
            raise Exception("Wrist digital write error")
