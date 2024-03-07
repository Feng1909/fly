#! /usr/bin/python3.8
import rospy

from airsim_ros.msg import VelCmd, CirclePoses
from nav_msgs.msg import Odometry
from airsim_ros.srv import Takeoff
from geometry_msgs.msg import PoseStamped
from math import atan2, hypot, cos, sin
from sensor_msgs.msg import Imu

rospy.init_node("tracking")

ring_x = None
ring_y = None
ring_area = None
target_x = None
target_y = None
target_z = None
target_yaw = None
flag = False
takeoffflag = False
number = 0
last_det_time = None
pose_x = None
pose_y = None
pose_z = None
pose_w = None
distence = 999999
yaw_old = None
pre_err_y = 0
pre_err_z = 0
sort = None

takeoff_client = rospy.ServiceProxy('/airsim_node/drone_1/takeoff', Takeoff)

control_cmd_pub = rospy.Publisher("/airsim_node/drone_1/vel_cmd_body_frame", VelCmd, queue_size=1)

def vins_cb(msg: Odometry):
    global takeoffflag, ring_x, ring_y, ring_area, target_x, target_y, target_z, target_yaw, flag, number, last_det_time, distence, sort
    if not takeoffflag:
        # 调用起飞服务
        response = takeoff_client()
        while not response.success:
            response = takeoff_client()
        takeoffflag = 1
    cmd = VelCmd()

    if ring_area == None or target_x == None:
        return

    if ring_area > 2000:
        # print("area: ", ring_area)
        # print(ring_y)
        if last_det_time == None:
            # last_det_time = rospy.Time.now()
            return
        if not flag:
            print('flag turn True')
            flag = True
        last_det_time = rospy.Time.now()
        # pass
    matrix_c = 1-2*pow(msg.pose.pose.orientation.y, 2) - 2*pow(-msg.pose.pose.orientation.z, 2)
    matrix_s = 2*(msg.pose.pose.orientation.x)*(msg.pose.pose.orientation.y)+2*msg.pose.pose.orientation.w*msg.pose.pose.orientation.z
    if not flag:
        distence = hypot((target_x-msg.pose.pose.position.x)/10, (target_y+msg.pose.pose.position.y)/10)
        theta = -atan2((target_y+msg.pose.pose.position.y)/10, (target_x-msg.pose.pose.position.x)/10) - (-atan2(-matrix_s, matrix_c))
        # print("distence: ", distence, target_x, target_y, target_z)
        # print('theta: ', atan2((target_y+msg.pose.pose.position.y)/10, (target_x-msg.pose.pose.position.x)/10), (-atan2(-matrix_s, matrix_c)))
        cmd.twist.linear.x = distence*cos(theta)
        cmd.twist.linear.y = distence*sin(-theta)
        cmd.twist.linear.z = target_z+msg.pose.pose.position.z
        cmd.twist.angular.z = (target_yaw-atan2(-matrix_s, matrix_c)/3.14*180)/100
    else:
        cmd.twist.linear.x = 1.0
        cmd.twist.linear.y = -(320-ring_x)/300
        cmd.twist.linear.z = -(240-ring_y)/100
        cmd.twist.angular.z = (target_yaw-atan2(-matrix_s, matrix_c)/3.14*180)/100
        
    if flag and rospy.Time.now()-last_det_time>rospy.Duration(3.0):
        flag = False
        number += 1
        print("Going to number: ", sort[number])

    control_cmd_pub.publish(cmd)

def pose_cb(msg: PoseStamped):
    global takeoffflag, ring_x, ring_y, ring_area, target_x, target_y, target_z, target_yaw, flag, number, last_det_time, pose_x, pose_y, pose_z, pose_w, distence, yaw_old, pre_err_y, pre_err_z, sort
    if not takeoffflag:
        # 调用起飞服务
        response = takeoff_client()
        while not response.success:
            response = takeoff_client()
        takeoffflag = 1
    cmd = VelCmd()

    # if ring_area == None or target_x == None:
    #     return

    if last_det_time == None:
        last_det_time = rospy.Time.now()
        return
    
    matrix_c = 1-2*pow(pose_y, 2) - 2*pow(pose_z, 2)
    matrix_s = 2*(pose_x)*(pose_y)+2*pose_w*pose_z
    yaw = atan2(-matrix_s, matrix_c)
    if yaw_old == None:
        yaw_old = yaw
    else:
        while (yaw-yaw_old) > 3.14/2:
            yaw -= 3.14
        while (yaw-yaw_old) < -3.14/2:
            yaw += 3.14
    yaw_old = yaw
    
    target_yaw_ = target_yaw
    
    distence = hypot((target_x-msg.pose.position.x)/10, (target_y-msg.pose.position.y)/10)
    theta = (-atan2((target_y-msg.pose.position.y)/10, (target_x-msg.pose.position.x)/10) - yaw)

    if sort[number] < 0 and distence+(target_z - msg.pose.position.z)**2 < 1:
        number += 1
        print("Going to number: ", sort[number])

    if not flag:
        cmd.twist.linear.x = distence*cos(theta)*1.5
        cmd.twist.linear.y = distence*sin(-theta)*3.0
        cmd.twist.linear.z = target_z-msg.pose.position.z
        cmd.twist.angular.z = (target_yaw_+yaw/3.14*180)/100
    else:
        cmd.twist.linear.x = 1.5
        cmd.twist.linear.y = -(320-ring_x)*0.01 + (-(320-ring_x) - pre_err_y)*0.05
        cmd.twist.linear.z = -(240-ring_y)*0.01 + (-(240-ring_y) - pre_err_z)*0.05
        pre_err_y = -(320-ring_x)
        pre_err_z = -(240-ring_y)
        cmd.twist.angular.z = (target_yaw_+yaw/3.14*180)/100
    
    # if last_det_time != None:
        # print('time: ', rospy.Time.now()-last_det_time)
    if flag and rospy.Time.now()-last_det_time>rospy.Duration(1.5):
        flag = False
        number += 1
        print("Going to number: ", sort[number])

    control_cmd_pub.publish(cmd)


def yolo_cb(msg: Info):
    global ring_x, ring_y, ring_area, last_det_time, flag, distence, sort, number
    if msg.area > 2000:
        ring_x = msg.x
        ring_y = msg.y
        ring_area = msg.area
    # print(distence)
    if sort == None:
        return
    # print(sort[number])
    if msg.area > 2000 and distence < 0.7 and sort[number] >= 0:
        last_det_time = rospy.Time.now()
        if not flag:
            print('flag turn True')
            flag = True

def circle_cb(msg: CirclePoses):
    global target_x, target_y, target_z, target_yaw, number, sort
    sort = [0, 1, 2, 3, -4, 4, 5, 6, 8, -1, 12, 13, 14, -2, -3, 15, 16]
    # sort = [14, -2, -3, 15, 16]
    number = min(number, len(sort)-1)
    # number = 4
    # print(msg.poses[number].yaw)
    if sort[number] == -1:
        target_x = -9.57481575012207
        target_y = 49.34465789794922
        target_z = -16.629619598388672
    elif sort[number] == -2:
        # x: -14.00959587097168
        # y: 126.54122924804688
        # z: -36.986141204833984

        target_x = -46
        target_y = 119
        target_z = -38.671329498291016
    elif sort[number] == -3:
        target_x = -30
        target_y = 130
        target_z = -35
    elif sort[number] == -4:
        target_x = 103
        target_y = 30
        target_z = -3
    else:
        target_x = msg.poses[sort[number]].position.x-4*cos(msg.poses[sort[number]].yaw/180*3.14)
        target_y = msg.poses[sort[number]].position.y-4*sin(msg.poses[sort[number]].yaw/180*3.14)
        target_z = msg.poses[sort[number]].position.z
        if sort[number] == 4 or sort[number] == 5 or sort[number] == 6 or sort[number] == 8 or sort[number] == 12:
            target_yaw = msg.poses[sort[number]].yaw+180
            target_x = msg.poses[sort[number]].position.x-4*cos(target_yaw/180*3.14)
            target_y = msg.poses[sort[number]].position.y-4*sin(target_yaw/180*3.14)
        # elif :
        #     target_yaw = msg.poses[sort[number]].yaw-180
        #     target_x = msg.poses[sort[number]].position.x-4*cos(target_yaw/180*3.14)
        #     target_y = msg.poses[sort[number]].position.y-4*sin(target_yaw/180*3.14)
        else:
            target_yaw = msg.poses[sort[number]].yaw

def imu_cb(msg: Imu):
    global pose_x, pose_y, pose_z, pose_w
    pose_x = msg.orientation.x
    pose_y = msg.orientation.y
    pose_z = msg.orientation.z
    pose_w = msg.orientation.w

# rospy.Subscriber("/vins_fusion/imu_propagate", Odometry, vins_cb, queue_size=1)
rospy.Subscriber("/airsim_node/drone_1/pose", PoseStamped, pose_cb, queue_size=1)
rospy.Subscriber("/airsim_node/drone_1/detect_result_out", Info, yolo_cb, queue_size=1)
rospy.Subscriber("/airsim_node/drone_1/circle_poses", CirclePoses, circle_cb, queue_size=1)
rospy.Subscriber("/airsim_node/drone_1/imu/imu", Imu, imu_cb, queue_size=1)

rospy.spin()