#!/usr/bin/env python

import numpy as np
from math import pi, cos, sin
import modern_robotics as mr

def forward_kinematics(joints):
    #input: joint angles [joint1, joint2, joint3]
    #output: the position of the end effector [x,y,z]
    #add your code here to complete the computation


    ##input

    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]
    joint4 = joints[3]	
  
    #Find the Home Position of the robotic arm
    M = np.array([[0,-1,0,-83.5],
			[1,0,0,195],
			[0,0,1,50], 
			[0,0,0,1]])

    w1 = np.array([0,0,1])
    w2 = np.array([0,0,0])
    w3 = np.array([0,0,0])
    w4 = np.array([0,0,1])

    q1 = np.array([0,0,0])
    q4 = np.array([-83.5,195,50])

    v1 = np.cross(-w1, q1)
    v2 = np.array([0,0,1])
    v3 = np.array([0,1,0])
    v4 = np.np.cross(-w4, q4)

    s1 = np.concatenate([w1, v1])
    s2 = np.concatenate([w2, v2])
    s3 = np.concatenate([w3, v3])
    s4 = np.concatenate([w4, v4])

    #Screw axis for joint 4	
    e4= mr.VecTose3(s4) * joint4
    exp_S4 = mr.MatrixExp6(e4)
    

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
    T = np.matmul(T, exp_S4)
    T = np.matmul(T, M)
   
    x = T[0][3]
    y = T[1][3]
    z = T[2][3]
    print([x, y, z])
    print('hello')
    return [x, y, z]

