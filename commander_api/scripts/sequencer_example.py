#!/usr/bin/env python3
import rospy
import math
from commander_api.sequencer_client import SequencerClient
import time

pi = math.pi

rospy.init_node('robot_example')
sequence = SequencerClient()

# #start main sequence
# sequence.start()

# #start a function in blockly called 'foo'
# sequence.start('foo')

# #start a function in blockly called 'foo' with arg x = 12
# sequence.start('foo with args',['x'],['12'])

# #pause sequence
# sequence.pause()

# #stop sequence
# sequence.stop()
time.sleep(1)
#load an existing sequnce
# sequence.load_project('xtura')
state=sequence.get_variable('counter') 
print(state)
res = sequence.get_variables_names()
print(res)