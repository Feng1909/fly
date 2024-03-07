#! /usr/bin/python3.8

import numpy as np
import casadi as ca
import time
from math import sqrt, acos, asin, atan2

# ROS
import rospy

from ius_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
# from mavros_msgs.msg import State
# from mavros_msgs.srv import CommandBool, SetMode
from airsim_ros.msg import VelCmd
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

import os, sys

class Traj():
    def __init__(self, traj : Trajectory):
        poss = []
        yaws = []
        ts = []
        for i, pos in enumerate(traj.pos):
            poss.append([pos.x, pos.y, pos.z])
            yaws.append(traj.yaw[i])
            ts.append(traj.time[i])

        self._poss = np.array(poss)
        self._yaws = np.array(yaws)
        self._N = self._poss.shape[0]
        if self._N < 2:
            return
        dir = self._poss[1 : ] - self._poss[ : -1]
        # _dir_norm是每个点的方向向量的模
        self._dir_norm = np.linalg.norm(dir, axis = 1)
        # _u_dir是每个点的方向向量
        self._u_dir = dir/self._dir_norm[:, np.newaxis]
        self._ts = np.array(ts)

    def sample(self, pos, dt, N):
        # calculate t0 and idx0
        t0 = 0
        idx0 = 0
        
        pos = np.array(pos)
        dl = np.linalg.norm(self._poss - pos, axis=1) # 求每个点到当前位置的距离
        min_idx = np.argmin(dl) # 索引的最小值
        # 如果min_idx == 0，说明当前位置在第一个点之前，那么t0就是第一个点的时间
        if min_idx == 0:
            idx0 = min_idx
            d_v = pos - self._poss[0]
            u_dir = self._u_dir[0]
            u_t =  np.dot(d_v, u_dir) / self._dir_norm[0]
            if u_t < 0:
                t0 = self._ts[0]
            else:
                t0 = u_t * (self._ts[1] - self._ts[0]) + self._ts[0]
        else:
            idx0 = min_idx - 1
            d_v = pos - self._poss[idx0]
            u_dir = self._u_dir[idx0]
            u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
            if u_t > 1:
                idx0 = idx0 + 1
                if idx0 == self._N - 1:
                    t0 = self._ts[-1]
                else:
                    d_v = pos - self._poss[idx0]
                    u_dir = self._u_dir[idx0]
                    u_t = np.dot(d_v, u_dir) / self._dir_norm[idx0]
                    if u_t < 0:
                        t0 = self._ts[idx0]
                    else:
                        t0 = u_t * (self._ts[idx0 + 1] - self._ts[idx0]) + self._ts[idx0]
            else:
                t0 = u_t * (self._ts[idx0 + 1] - self._ts[idx0]) + self._ts[idx0]

        # sample N points
        ts = np.linspace(t0 + dt, t0 + dt * N, N)
        idx = idx0
        poss = []
        yaws = []
        for t in ts:
            while idx + 1 < self._N and t > self._ts[idx + 1]:
                idx += 1
            if idx == self._N - 1:
                poss.append(self._poss[-1])
                yaws.append(self._yaws[-1])
                continue
            u_dir = self._u_dir[idx]
            u_t = (t - self._ts[idx]) / (self._ts[idx + 1] - self._ts[idx])
            poss.append(self._poss[idx] + u_t * self._dir_norm[idx] * u_dir)
            yaws.append(self._yaws[idx] + u_t * (self._yaws[idx + 1] - self._yaws[idx]))
        
        return np.array(poss), np.array(yaws), ts

trajectory = None
rospy.init_node("tracking")

control_cmd_pub = rospy.Publisher("/airsim_node/drone_1/vel_cmd_body_frame", VelCmd, queue_size=1)

def track_traj_cb(msg: Trajectory):
    global trajectory
    trajectory = Traj(msg)

kp_x = 1
kp_y = kp_x
kp_z = 10
kd_x = 1
kd_y = kd_x
kd_z = 0
pre_x_error = 0
pre_y_error = pre_x_error
pre_z_error = 0

def vins_cb(msg: Odometry):
    global kp_x, kp_y, kp_z, kd_x, kd_y, kd_z, pre_x_error, pre_y_error, pre_z_error
    if trajectory != None:
        p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        pose_z = msg.pose.pose.position.z
        
        poss, yaws, ts = trajectory.sample(p, 0.1, 5)

        poss = poss.tolist()[4]
        error_x = poss[0] - pose_x
        error_y = poss[1] - pose_y
        error_z = poss[2] - pose_z
        # print(poss.tolist()[0])
        p_x = kp_x * error_x
        p_y = kp_y * error_y
        p_z = kp_z * error_z

        d_x = (error_x - pre_x_error) / 0.1 * kd_x
        d_y = (error_y - pre_y_error) / 0.1 * kd_y
        d_z = (error_z - pre_z_error) / 0.1 * kd_z
        pre_x_error = error_x
        pre_y_error = error_y
        pre_z_error = error_z

        u = VelCmd()
        u.twist.linear.x = p_x + d_x
        u.twist.linear.y = -(p_y + d_y)
        u.twist.linear.z = -(p_z + d_z)

        control_cmd_pub.publish(u)
        # print('target: ', poss)
        print('error: ', round(error_x, 4),
                         round(error_y, 4),
                         round(error_z, 4))
        print(round(u.twist.linear.x, 4),
              round(u.twist.linear.y, 4),
              round(u.twist.linear.z, 4))

rospy.Subscriber("~track_traj", Trajectory, track_traj_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("/vins_fusion/odometry", Odometry, vins_cb, queue_size=1)

rospy.spin()
