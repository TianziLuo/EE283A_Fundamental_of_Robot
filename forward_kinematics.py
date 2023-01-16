#!/usr/bin/env python

import numpy as np
from math import pi, cos, sin
import modern_robotics as mr

def forward_kinematics(joints):
    #input: joint angles [joint1, joint2, joint3]
    #output: the position of the end effector [x,y,z]
    #add your code here to complete the computation


    ##input

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150

    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]
  
    #Find the Home Position of the robotic arm
    M = np.array([[1,0,0,link3x + link4x],
			[0,1,0,0],
			[0,0,1,link1z + link2z + link3z], 
			[0,0,0,1]])
   
    w1 = np.array([0,0,1])
    w2 = np.array([0,1,0])
    w3 = np.array([0,-1,0])

    q1 = np.array([0,0,link1z])
    q2 = np.array([0,0,link1z+link2z])
    q3 = np.array([link3x,0,link3z+link1z+link2z])

    v1 = np.cross(-w1, q1)
    v2 = np.cross(-w2, q2)
    v3 = np.cross(-w3, q3)

    s1 = np.concatenate([w1, v1])
    s2 = np.concatenate([w2, v2])
    s3 = np.concatenate([w3, v3])

    #Screw axis for joint 3	
    e3= mr.VecTose3(s3) * joint3
    exp_S3 = mr.MatrixExp6(e3)
    

    #Screw axis for joint 2	
    e2 = mr.VecTose3(s2) * joint2
    exp_S2 = mr.MatrixExp6(e2)
    
   
    #Screw axis for joint 1	
    e1 = mr.VecTose3(s1) * joint1 
    exp_S1 = mr.MatrixExp6(e1)

    T = np.matmul(exp_S1, exp_S2)
    T = np.matmul(T, exp_S3)
    T = np.matmul(T, M)
   
    x = T[0][3]
    y = T[1][3]
    z = T[2][3]

    return [x, y, z]
