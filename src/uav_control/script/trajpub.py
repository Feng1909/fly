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

rospy.init_node("traj_pub")
rospy.loginfo("traj_pub is OK")

BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/function_model/ref_traj/'
sys.path += [BASEPATH]

trajectory_pub = rospy.Publisher("~track_traj", Trajectory, tcp_nodelay=True, queue_size=1)

# create the csv file which record the reference path 
total_length = 1000
def _ref_traj_create(file_name):
    global total_length
    with open(BASEPATH + file_name + '.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["time", "px", "py", "pz", "yaw"])
        for i in range(total_length):
            writer.writerow([1*i/10, 0, 0, i, 0])
                
# read the csv and opereate the path, get the path_points and publish them
def _ref_traj_pub(file_name, time_length):
    traj_point = Trajectory()
    pose = Point()
    
    cnt = 0
    trajref = []
    with open(BASEPATH + file_name + ".csv", 'r') as file:
        csvreader = csv.reader(file)
        next(csvreader)
        for row in csvreader:
            trajref += [row] 
            
    t = time.time()
    last_t = t
    for i in range(total_length - time_length):
        for j in range(time_length):
            # pose.x = trajref[i + j][1]
            # pose.y = trajref[i + j][2]
            # pose.z = trajref[i + j][3]
            
            # traj_point.time += [trajref[i + j][0]]
            # traj_point.pos.append(pose)
            # traj_point.yaw += [trajref[i + j][4]]
            
            pose.x = 0
            pose.y = 0
            pose.z = i/10
            traj_point.pos.append(pose)
            traj_point.time += [i/100]
            traj_point.yaw += [i/1000]
            
            # print("$$$$$$$$$", pose)
            
            # print(traj_point.pos)
            # print(i+j)
            # print("**")
            # pos.x = 0
            # pos.y = 0
            # pos.z = 3
            
            # traj_point.time += [0]
            # traj_point.pos += [pos]
            # traj_point.yaw += [0]
        # print(traj_point)
        # publish traj_point every 0.1 sec
        t = time.time()
        while t - last_t < 0.1:
            t = time.time()
            
        trajectory_pub.publish(traj_point)  
        print("ok")      
        last_t = t
        traj_point.time.clear()
        traj_point.pos.clear()
        traj_point.yaw.clear()   
        print("@@",traj_point.pos)     
         
def _ref_traj_pub2():
    pose = Point()
    traj_point = Trajectory()

    t = time.time()
    last_t = t
    for i in range(1000):
        for j in range(10):
            pose.x = 0
            pose.y = 0
            pose.z = 5
            traj_point.pos.append(pose)
            traj_point.time += [i/10 + j/10]
            traj_point.yaw += [0]
        
        t = time.time()
        while t - last_t < 0.1:
            t = time.time()
        trajectory_pub.publish(traj_point)
        # print("traj_pub!", traj_point)
        last_t = t
        traj_point.time.clear()
        traj_point.pos.clear()
        traj_point.yaw.clear()   
if __name__ == "__main__":
    _ref_traj_create("traj_pub")
    time_length = 30
    # _ref_traj_pub("traj_pub", time_length)
    _ref_traj_pub2()
    