#!/usr/bin/env python3
import PyKDL

class TransformFrame:
    """Transform frame class with x,y,z position and rx, ry,rz rotation atributes
    """        
    def __init__(self):      
        self.x =0.0
        self.y =0.0
        self.z =0.0
        self.rx =0.0
        self.ry =0.0
        self.rz =0.0

    def get_quaternion(self):
        rot = PyKDL.Rotation.EulerZYX(self.rz, self.ry, self.rx)
        quat = rot.GetQuaternion()
        return [quat[0],quat[1],quat[2],quat[3]]

    def from_quaternion(self, q1, q2, q3, q4):
        rot = PyKDL.Rotation.Quaternion(q1, q2, q3, q4)
        euler = rot.GetEulerZYX()
        self.x = euler[2]
        self.y = euler[1]
        self.z = euler[0]

class ErrorMessage:
    """Error message class with error code, source and message attributes
    """    
    def __init__(self):
        self.error_code = 0
        self.source = ""
        self.message = ""

        
        