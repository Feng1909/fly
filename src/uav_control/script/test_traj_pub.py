#! /usr/bin/python3.8

import numpy as np
import time

# ROS
import csv
import rospy

from ius_msgs.msg import Trajectory
from geometry_msgs.msg import Point
#
import os, sys

from math import cos, sin

rospy.init_node("test_traj_pub")

BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/function_model/ref_traj/'
sys.path += [BASEPATH]

trajectory_pub = rospy.Publisher("/ius_uav/trajectory", Trajectory, tcp_nodelay=True, queue_size=1)

def traj_pub():
    points = [[23.879, 9.893, -0.8659],
              [47.221, 5.750, -12.143],
              [88.608, 8.812, -1.4570],
              [100.74, 23.32, -11.936],
              [76.058, 45.84, -11.838],
              ]
    vels = [[0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            ]
    acc = [[0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
           ]
if __name__ == "__main__":
    
    # timer
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        traj_pub()
        rate.sleep()