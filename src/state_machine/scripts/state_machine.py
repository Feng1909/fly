#! /usr/bin/env python3
#coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry, Path
# import time
from std_msgs.msg import Int8
import numpy as np

class Algorithm:
    
    def __init__(self):
        rospy.init_node("arcuo_det")
        self.load_params()
        self.odom = Odometry()
        self.mavros_state = State()
        self.aruco_pose = []
        self.target_point = PoseStamped()
        self.aruco_local = PoseStamped()
        self.car_detected = False
        self.aruco_detected = False

        self.debug_jump_to = 9

        self.arucos = []
        
        '''
        0: Arming
        1: Taking off
        2: Acrossing the 1st square
        3: Going to the 2nd square
        4: Acrossing the 2nd square
        5: Going to the car
        6: Recognizing the car
        7: Going to land
        8: Recognizing the Aruco code
        9: Landing
        10: Disarming
        11: Finish
        '''
        self.state = 0

        self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)
        self.aruco_sub = rospy.Subscriber("/aruco_det/target_loc", PoseStamped, self.aruco_callback)
        self.aruco_local_sub = rospy.Subscriber("/aruco_det/target_loc_local", PoseStamped, self.aruco_local_callback)
        self.mavros_state_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)

        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.target_point_pub = rospy.Publisher("/waypoint_generator/waypoints", Path, queue_size=10)

        self.state_machine_state_pub = rospy.Publisher("/state_machine", Int8, queue_size=10)

        self.vel_cmd_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
    
    def odom_callback(self, msg):
        self.odom = msg
    
    def aruco_local_callback(self, msg: PoseStamped):
        self.aruco_local = msg

    def aruco_callback(self, msg: PoseStamped):
        if self.state != 8 or self.aruco_detected == True:
            return
        self.arucos.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if len(self.arucos) == 5:
            aruco_tmp = np.array(self.arucos)
            # if max(aruco_tmp[:,0])
            if max(aruco_tmp[:,0]) - min(aruco_tmp[:,0]) < 0.1 and \
               max(aruco_tmp[:,1]) - min(aruco_tmp[:,1]) < 0.1 and \
                max(aruco_tmp[:,2]) - min(aruco_tmp[:,2]) < 0.1:
                self.aruco_detected = True
                self.aruco_pose = [msg.pose.position.x, msg.pose.position.y, 0.05]
                # self.aruco_pose.pose.position.z = 0.0
                print(self.aruco_pose)
            self.arucos = self.arucos[1:]
    
    def mavros_state_callback(self, msg):
        self.mavros_state = msg
    
    def is_close(self, odom: Odometry, pose: list):
        # print(abs(odom.pose.pose.position.x - pose[0]), abs(odom.pose.pose.position.y - pose[1]))
        if abs(odom.pose.pose.position.x - pose[0]) < 0.05 and abs(odom.pose.pose.position.y - pose[1]) < 0.05 and abs(odom.pose.pose.position.z - pose[2]) < 0.05:
            return True
        return False

    def go_to(self, pose: list):
        path_to_pub = Path()
        path_to_pub.poses.clear()
        self.target_point.pose.position.x = pose[0]
        self.target_point.pose.position.y = pose[1]
        self.target_point.pose.position.z = pose[2]
        path_to_pub.poses.append(self.target_point)
        # print(path_to_pub)
        self.target_point_pub.publish(path_to_pub)

    def run(self):
        # Arming
        print('state: ', self.state)
        state_now = Int8()
        state_now.data = self.state
        self.state_machine_state_pub.publish(state_now)
        if self.state == 0:
            if self.mavros_state.armed == True or self.debug_jump_to > 0:
                self.state = 1
                return
            self.arming_client.call(True)
            return
        
        # Taking off
        elif self.state == 1:
            # print(self.takeoff_point)
            # print(self.odom.pose.pose.position)
            if self.debug_jump_to > 1 or self.is_close(self.odom, self.takeoff_point):
                self.state = 2
                return
            else:
                self.go_to(self.takeoff_point)
                return 
        
        # Acrossing the 1st square
        elif self.state == 2:
            if self.debug_jump_to > 2 or self.is_close(self.odom, self.first_square_point):
                self.state = 3
                return
            else:
                self.go_to(self.first_square_point)
                return
        
        # Going to the 2nd square
        elif self.state == 3:
            if self.debug_jump_to > 3 or self.is_close(self.odom, self.second_square_point_pre):
                self.state = 5
                return
            else:
                self.go_to(self.second_square_point_pre)
                return
        
        # Acrossing the 2nd square
        elif self.state == 4:
            if self.debug_jump_to > 4 or self.is_close(self.odom, self.second_square_point_end):
                self.state = 5
                return
            else:
                self.go_to(self.second_square_point_end)
                return
        
        # Going to the car
        elif self.state == 5:
            if self.debug_jump_to > 5 or self.is_close(self.odom, self.car_point):
                self.state = 6
                return
            else:
                self.go_to(self.car_point)
                return
        
        # Recognizing the car
        elif self.state == 6:
            if self.debug_jump_to > 6 or self.car_detected == False:
                self.state = 7
                return
            return
        
        # Going to land
        elif self.state == 7:
            if self.debug_jump_to > 7 or self.is_close(self.odom, self.land_point):
                self.state = 8
                return
            else:
                self.go_to(self.land_point)
                return
            
        # Recognizing the Aruco code
        elif self.state == 8:
            if self.debug_jump_to > 8 or self.aruco_detected == True:
                self.state = 9
                return
            return
        
        # Landing
        elif self.state == 9:
            if self.debug_jump_to > 9 or self.odom.pose.pose.position.z < -10.3:
                self.state = 10
                return
            else:
                # self.go_to(self.aruco_pose)
                vel_cmd = Twist()
                vel_cmd.linear.z = 0.05
                vel_cmd.linear.x = 0
                vel_cmd.linear.y = 0
                # vel_cmd.linear.x = max(min((self.aruco_local.pose.position.x-320)/500, 0.1), -0.1)
                # vel_cmd.linear.y = max(min(-(self.aruco_local.pose.position.y-240)/500, 0.1), -0.1)
                self.vel_cmd_pub.publish(vel_cmd)
                return
        
        # Disarming
        elif self.state == 10:
            if self.debug_jump_to > 10 or self.mavros_state.armed == False:
                self.state = 11
                return
            # self.arming_client.call(False)
            # disarm_command = CommandBoolRequest(value=False)
            # self.arming_client.call(disarm_command)
            self.set_mode_client(0,'AUTO.LAND')
            return
        
        # Finish
        elif self.state == 11:
            return

    def load_params(self):

        self.debug = rospy.get_param('~is_debug')
        self.takeoff_point = rospy.get_param('~takeoff')
        self.first_square_point = rospy.get_param('~1st_square')
        self.second_square_point_pre = rospy.get_param('~2nd_square_pre')
        self.second_square_point_end = rospy.get_param('~2nd_square_end')
        self.car_point = rospy.get_param('~car')
        self.land_point = rospy.get_param('~land')
        print(type(self.land_point))

if __name__ == "__main__":

    main_algorithm = Algorithm()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        main_algorithm.run()
        rate.sleep()