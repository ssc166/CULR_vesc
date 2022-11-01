#!/usr/bin/env python

import sympy as sp
import numpy as np
import math
import time
import rospy
import os
from std_msgs.msg import Float64, Float64MultiArray
from gazebo_msgs.srv import GetModelState, GetLinkState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
import pylab as pl
import control
import matplotlib.pyplot as plt
import state_equation_roll as ser
import WIP_utils as utils

DEG2RAD = np.pi/180
RAD2DEG = 180/np.pi
# 0.5004170132265042
z_com = 0.436452
A, B, C, D = ser.Cal_Roll_SS(z_com, 2000)
Q = sp.Matrix([ [1,    0,    0,    0],
                [0,    10,    0,    0],
                [0,    0,    1,    0],
                [0,    0,    0,    1]])

R = sp.Matrix([ [20] ])

    
def roll_K_gain_R(A, B, Q, R):
        
    K, S, E = control.lqr(A, B, Q, R)
    
    return K, S, E

K, S, E = roll_K_gain_R(A, B, Q, R)
print('K: ', K)

def cmg_lqr(pos_gb_data, rps_gb_data, imu_cmg_data):

    
    
    x0 = np.array([imu_cmg_data[0][0], pos_gb_data, imu_cmg_data[0][1], rps_gb_data], dtype=object)
    
    V = - K@x0
    
    print(x0)
    print(V)
    print('==============================')
    # gimbal_ang = x_next[1]*RAD2DEG
    # x_next =  - K @ x0
    return V
    
