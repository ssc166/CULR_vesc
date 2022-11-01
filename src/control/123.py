#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import Float64, Float64MultiArray, String
from gazebo_msgs.srv import GetModelState, GetLinkState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import *
import pylab as pl
import matplotlib.pyplot as plt
import state_equation_roll as ser
import WIP_utils as utils
from math import nan
from tkinter.constants import NONE
import matplotlib.pyplot as plt
import numpy as np
from openrobot_vesc_pyserial import vesc_pyserial as vs
from threading import Timer
from tkinter.font import ROMAN
from itertools import count
import mpc_pitch
import Cal_joint as cj

def Imucallback(msg):
    # global roll
    # global roll_vel

    # roll, pitch, yaw = utils.quaternion_to_euler_angle(msg.orientation) 
    # roll_vel = msg.angular_velocity.x
    if msg == 1:
        print(10)
    else:
        print(20)
    msg = 0
    

while True:

    rospy.init_node('Imu_read', anonymous=True)
    # imu_sub = rospy.Subscriber('/imu/data_raw', Imu, Imucallback)
    imu_sub = rospy.Subscriber('chatter', Float64, Imucallback)


    rospy.spin()