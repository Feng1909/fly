#! /usr/bin/python3.8

import numpy as np
import casadi as ca
import time

# ROS
import rospy

from ius_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from mavros_msgs.msg import AttitudeTarget
# from geometry_msgs.msg import body_rate, thrust
from mpl_toolkits.mplot3d import Axes3D
# 
from uav_control.msg import Thrust_w
#
import os, sys

BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/function_model/'
sys.path += [BASEPATH]

from quadrotor_control import QuadrotorModel
from tracker import TrackerMPC
from trajectory import Trajectory_ref, StateSave

# from plotting import plot_gates_2d, plot_traj_xy

rospy.init_node("track_mpc")
rospy.loginfo("ROS: Hello")

# traj = Trajectory(BASEPATH+"results/res_t_n6.csv")
traj = Trajectory_ref()
quad =  QuadrotorModel(BASEPATH+'quad/quad_px4.yaml')

tracker = TrackerMPC(quad)

# tracker.define_opt()
tracker.load_so(BASEPATH+"generated/track_mpc.so")

state_saver = StateSave(BASEPATH+"results/flight1.csv")

imu_data = Imu()

ctrl_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, tcp_nodelay=True, queue_size=1)

stop_flag = False
def stop_cb(msg: Bool):
    global stop_flag
    stop_flag = msg.data

r_x = []
r_y = []
r_z = []
# last_t = time.time()
cnt = 0
last_t = time.time()
time_factor = 1
def odom_cb(msg: Odometry):
    global cnt, time_factor, last_t
    print("odom_cb!")
    if cnt == 0:
        traj.sample_dt_reset()
        last_t = time.time()
    
    cnt += 1
    # if time_factor < 0.5 and time_factor >= 0.001:
    #     time_factor += 0.001
    # else:
    #     time_factor = 0.5
     
    if traj._N != 0:
        x0 = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                    msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        if cnt > 0 :
            r_x.append(msg.pose.pose.position.x)
            r_y.append(msg.pose.pose.position.y)
            r_z.append(-msg.pose.pose.position.z)
        
        tim = time.time()
        traj_p, traj_yaw = traj.sample((tim-last_t)*time_factor, 0.1*time_factor, 5)
        last_t = tim
        print("*********mpc_param*********")
        print("\nuav_x:",x0)
        print("\nmpc_trajp:",traj_p)
        print("\nmpc_trajyaw:",traj_yaw)
        print("===========================")
        # x0, p*herizon, yaw*herizon 
        res = tracker.solve(x0, traj_p.reshape(-1), traj_yaw.reshape(-1))
        
        x = res['x'].full().flatten()
        Tt = 1*(x[tracker._Herizon*13+0]+x[tracker._Herizon*13+1]+x[tracker._Herizon*13+2]+x[tracker._Herizon*13+3])
        
        u = AttitudeTarget()
        u.thrust = Tt/4/quad._T_max
        u.body_rate.x = x[10]
        u.body_rate.y = x[11]
        u.body_rate.z = x[12]
        print("########mpc_u########")
        print("mpc_u:",u)
        print("---------------------")
        ctrl_pub.publish(u)

        # data save
        state_saver.log(time.time(), x0, [u.thrust, u.body_rate.x, u.body_rate.y, u.body_rate.z], [0,0,0])

def imu_cb(msg:Imu):
    global imu_data
    imu_data = msg

def track_traj_cb(msg: Trajectory):
    print("track_traj_cb!")
    pos = []
    yaw = []
    time = []
    for i in range(len(msg.time)):
        pos.append([msg.pos[i].x, msg.pos[i].y, msg.pos[i].z])
        yaw.append(msg.yaw[i])
        time.append(msg.time[i])
       
    pos.append([msg.pos[-1].x, msg.pos[-1].y, msg.pos[-1].z])
    yaw.append(msg.yaw[-1])
    time.append(msg.time[-1])
    traj.load_data(np.array(pos), np.array(yaw), np.array(time))
   

rospy.Subscriber("~odom", Odometry, odom_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("~imu", Imu, imu_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("~track_traj", Trajectory, track_traj_cb, queue_size=1, tcp_nodelay=True)
rospy.Subscriber("stop_flag", Bool, stop_cb)

rospy.spin()
rospy.loginfo("ROS: Goodby")