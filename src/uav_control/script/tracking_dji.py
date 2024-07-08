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
from airsim_ros.msg import AngleRateThrottle
from airsim_ros.srv import Takeoff
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

import os, sys

BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/function_model/'
sys.path += [BASEPATH]

from quadrotor_control import QuadrotorSimpleModel
from tracker import TrackerMPC

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
takeoffflag = False
debug_x = None
debug_y = None
debug_z = None
traj_old = None
stable_flag = False

rospy.init_node("tracking")

# auto_offboard = rospy.get_param('~auto_offboard', True)
# setpoint_raw_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1, tcp_nodelay=True)
control_cmd_pub = rospy.Publisher("/airsim_node/drone_1/angle_rate_throttle_frame", AngleRateThrottle, queue_size=1)

takeoff_client = rospy.ServiceProxy('/airsim_node/drone_1/takeoff', Takeoff)

# mavros_state = None
# def mavros_state_cb(msg: State):
#     global mavros_state
#     mavros_state = msg
# rospy.Subscriber("/mavros/state", State, mavros_state_cb, queue_size=1, tcp_nodelay=True)
# arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
# set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

# quad = QuadrotorSimpleModel(BASEPATH+'quad/quad_px4.yaml')
quad = QuadrotorSimpleModel(BASEPATH+'quad/quad_sim.yaml')
tracker = TrackerMPC(quad)

tracker.define_opt()
# tracker.load_so(BASEPATH+"generated/track_mpc.so")

def track_traj_cb(msg: Trajectory):
    global trajectory
    trajectory = Traj(msg)

def odom_cb(msg: PoseStamped):
    global debug_x, debug_y, debug_z
    debug_x = msg.pose.position.x
    debug_y = msg.pose.position.y
    debug_z = msg.pose.position.z

def vins_cb(msg: Odometry):
    global takeoffflag, debug_x, debug_y, debug_z, stable_flag, traj_old, trajectory
    if not takeoffflag:
        # 调用起飞服务
        response = takeoff_client()
        while not response.success:
            response = takeoff_client()
        takeoffflag = 1
    v_x = msg.twist.twist.linear.x
    v_y = -msg.twist.twist.linear.y
    v_z = -msg.twist.twist.linear.z
    
    pose_w = msg.pose.pose.orientation.x
    pose_x = -msg.pose.pose.orientation.w
    pose_y = msg.pose.pose.orientation.z
    pose_z = -msg.pose.pose.orientation.y
    
    u = AngleRateThrottle()
    u.rollRate = 0
    u.pitchRate = 0
    u.yawRate = 0
    u.throttle = 0

    if trajectory != None:

        q = np.array([pose_w, pose_x, pose_y, pose_z])
        v = np.array([v_x, v_y, v_z])
        p = np.array([debug_x, debug_y, debug_z])
        # p = np.array([msg.pose.pose.position.x, -msg.pose.pose.position.y, -msg.pose.pose.position.z])

        x0 = np.concatenate([p, v, q])
        
        poss, yaws, ts = trajectory.sample(p, 0.1, 5)
        print("##############################################")
        res = tracker.solve(x0, poss.reshape(-1), yaws.reshape(-1))
        x = res['x'].full().flatten()
        Tt = x[tracker._Herizon*10+0]
        wx = x[tracker._Herizon*10+1]
        wy = x[tracker._Herizon*10+2]
        wz = x[tracker._Herizon*10+3]

        u.rollRate = wx
        u.pitchRate = wy
        u.yawRate = wz
        u.throttle = -Tt/9.0
        control_cmd_pub.publish(u)
        print(round(u.throttle, 4),
            round(u.rollRate, 4),
            round(u.pitchRate, 4),
            round(u.yawRate, 4))
        print(poss)
        print('v: ', v)
        print('q: ', q)

# rospy.Subscriber("~odom", Odometry, odom_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, odom_cb, queue_size=1)
# rospy.Subscriber("/airsim_node/drone_1/imu/imu", Imu, imu_cb, queue_size=1)
rospy.Subscriber("~track_traj", Trajectory, track_traj_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("/vins_fusion/imu_propagate", Odometry, vins_cb, queue_size=1)

rospy.spin()
