#! /usr/bin/env python3
#coding=utf-8

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool

class Controller:
    def __init__(self) -> None:
        rospy.init_node("setpoint")
        self.load_params()
        self.pub_point = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arm_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.sleep_time = 0.2

    def load_params(self):
        self.state = 0
        self.takeoff_point = rospy.get_param('~takeoff')
        self.first_square_point_pre = rospy.get_param('~1st_square_pre')
        self.first_square_point = rospy.get_param('~1st_square')
        self.first_square_point_end = rospy.get_param('~1st_square_end')
        self.second_square_point_pre = rospy.get_param('~2nd_square_pre')
        self.second_square_point_end = rospy.get_param('~2nd_square_end')
        self.car_point = rospy.get_param('~car')
        self.land_point = rospy.get_param('~land')
        self.land_point_end = rospy.get_param('~land_end')
        print(type(self.land_point))

    def pub(self, point: list):
        target_pose = PoseStamped()
        target_pose.pose.position.x = point[0]
        target_pose.pose.position.y = point[1]
        target_pose.pose.position.z = point[2]
        target_pose.pose.orientation.w = 1
        target_pose.pose.orientation.x = 0
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        self.pub_point.publish(target_pose)

    def is_close(self, odom: Odometry, point: list):
        if (odom.pose.pose.position.x - point[0])**2 + \
           (odom.pose.pose.position.y - point[1])**2 + \
           (odom.pose.pose.position.z - point[2])**2 < 0.2:
            return True
        return False 

    def odom_cb(self, msg: Odometry):
        print(self.state)
        if self.state == 0:
            if self.is_close(msg, self.takeoff_point):
                rospy.sleep(self.sleep_time)
                self.state = 1
            else:
                self.pub(self.takeoff_point)
        if self.state == 1:
            if self.is_close(msg, self.first_square_point_pre):
                rospy.sleep(self.sleep_time)
                self.state = 2
            else:
                self.pub(self.first_square_point_pre)
        if self.state == 2:
            if self.is_close(msg, self.first_square_point):
                rospy.sleep(self.sleep_time)
                self.state = 3
            else:
                self.pub(self.first_square_point)
        if self.state == 3:
            if self.is_close(msg, self.first_square_point_end):
                rospy.sleep(self.sleep_time)
                self.state = 4
            else:
                self.pub(self.first_square_point_end)
        if self.state == 4:
            if self.is_close(msg, self.second_square_point_pre):
                rospy.sleep(self.sleep_time)
                self.state = 5
            else:
                self.pub(self.second_square_point_pre)
        if self.state == 5:
            if self.is_close(msg, self.second_square_point_end):
                rospy.sleep(self.sleep_time)
                self.state = 6
                pass
            else:
                self.pub(self.second_square_point_end)
        if self.state == 6:
            if self.is_close(msg, self.car_point):
                rospy.sleep(self.sleep_time)
                self.state = 7
                pass
            else:
                self.pub(self.car_point)
        if self.state == 7:
            if self.is_close(msg, self.land_point):
                rospy.sleep(self.sleep_time)
                self.state = 8
                pass
            else:
                self.pub(self.land_point)
        if self.state == 8:
            if self.is_close(msg, self.land_point_end):
                rospy.sleep(self.sleep_time)
                self.state = 9
                pass
            else:
                self.pub(self.land_point_end)
        if self.state == 9:
            self.set_mode_client(0,'AUTO.LAND')
            rospy.sleep(self.sleep_time)
            self.arm_client(False)
            
        


controller = Controller()

rospy.Subscriber("~odom", Odometry, controller.odom_cb, queue_size=1, tcp_nodelay=True)

rospy.spin()