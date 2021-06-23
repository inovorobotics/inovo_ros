#!/usr/bin/env python3
import rospy
import math
from commander_api.robot_client import RobotClient
from commander_api.psu_client import  PSUClient
from commander_api.io_client import  IOClient
from commander_api.custom_datatypes import *

pi = math.pi

rospy.init_node('robot_example')
robot = RobotClient()
psu = PSUClient()
io = IOClient()

# psu.bus_power_off()

# psu.bus_power_on()

robot.disable()
robot.enable()



# #set digital output on the wrist to high
io.wrist_digital_write(channel = 0, value = 1)
wrist_value = io.wrist_digital_read(channel = 0)
# #read the digital input on the wrist
# wrist_value = robot.wrist_digital_read(pin = 0)
# print('Digital input on the wrist: ', wrist_value)

# #print the latest error message
error=robot.get_errors()
print(error.message)
# #set digital output on the psu to high
# psu.digital_write(pin=0, value = 1)
robot.clear_errors()
error=robot.get_errors()
print(error.message)
# #read the digital input of the psu
# psu_value = psu.dig
# ital_read(pin = 0)
# print('Digital input in the psu: ', psu_value)



#turn off the psu
# psu.power_off()