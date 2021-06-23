#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger
from sequencer.srv import *
from sequencer.msg import RuntimeState,Variable

class SequencerClient:
    """Class to control the sequencer
    """    
    def __init__(self):
        self.run_srv = rospy.ServiceProxy('/sequence/run', RunSequence)
        self.pause_srv = rospy.ServiceProxy('/sequence/pause', Trigger)
        self.stop_srv = rospy.ServiceProxy('/sequence/stop', Trigger)
        self.load_project_srv = rospy.ServiceProxy('/sequence/open_project', Project)
        self.get_var_srv = rospy.ServiceProxy('/sequence/get_var', GetVariable)
        self.runtime_state = int
        self.variable_names = []
        
    def start(self,name = '', variable_names= [], variable_values=[]):
        """Start the sequence or function

        :param name: Function name, defaults to the start block
        :type name: str, optional
        :param variable_names: Arguments to pass in the function, defaults to []
        :type variable_names: list, optional
        :param variable_values: Values of the arguments passed to the function, defaults to []
        :type variable_values: list, optional
        :raises Exception: Sequencer failed to start
        """         
        res=self.run_srv(name,variable_names,variable_values)
        if res.success == False:
            raise Exception("Sequencer failed to start")


    def pause(self):
        """Pause the running sequence

        :raises Exception: Sequence failed to pause
        """           
        res=self.pause_srv()
        if res.success == False:
            raise Exception("Sequence failed to pause")
            
    def stop(self):
        """Stop the running sequence

        :raises Exception: Sequence failed to stop
        """           
        res = self.stop_srv()
        if res.success == False:
            raise Exception("Sequence failed to stop")

    def load_project(self, name):
        """Load an existing project

        :param name: Project name
        :type name: string
        :raises Exception: Sequence failed to load a project
        """       
        res = self.load_project_srv(name)
        if res.success == False:
            raise Exception("Sequence failed to load a project")

    def get_sequencer_state(self):
        """Get the current state of the sequencer
            
        :return: Sequencer state, Idle = 0, Running = 1, Paused = 2
        :rtype: int
        """        
        msg = rospy.wait_for_message('/sequence/runtime_state', RuntimeState)
        return msg.state

    def set_variable(self, name, value):
        """Set a new variable in the sequence

        :param name: Name of the variable to set
        :type name: string
        :param value: Value of the new variable
        :type value: string
        :raises Exception: Failed to set a new variable
        """        
        res = self.set_var_srv(name, value)
        if res.success == False:
            raise Exception("Failed to set a new variable")

    def get_variable(self,name):
        """Get the value of the given variable if the sequence is running

        :param name: Name of the variable
        :type name: string
        :raises Exception: Failed to get the variable value
        :return: Value of the variable
        :rtype: string
        """
        res = self.get_var_srv(name)
        if res.success == False:
            raise Exception("Failed to get the variable value")
        return res.value

    def get_variables_names(self):
        """Get a list of variable names in the running sequence

        :return: List of variables names
        :rtype: list(string)
        """        
        msg = rospy.wait_for_message('/sequence/runtime_state', RuntimeState)
        for variable in msg.variables:
            self.variable_names.append(variable.name)
        return self.variable_names