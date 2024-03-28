#! /usr/bin/env python3
#coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from math import atan

class Algorithm:
    
    def __init__(self):
        rospy.init_node("arcuo_det")
        self.debug = rospy.get_param('~is_debug')
        self.num = 0
        self.state = Odometry()

        # for Aruco
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.detect = False

        # self.camera_Matrix = np.array([[404.035450, 0., 329.188263],
        #                                [0.,403.653476, 248.486988,],
        #                                [0., 0., 1. ]])
        self.camera_Matrix = np.array([[1.78634438e+03,0.00000000e+00,9.13022592e+02],
                                        [0.00000000e+00,1.82753404e+03,4.87987317e+02],
                                        [0.00000000e+00,0.00000000e+00,1.00000000e+00]])

        # self.distortion_Matrix = np.array([0.043602, -0.064718,
        #                             -0.000313, -0.000471,
        #                             0.])
        self.distortion_Matrix = np.array([4.51202586e-01,-2.62210378e+00,-5.92390289e-02,2.86948250e-03,4.59687027e+00])

        self.q_camera = np.array([0, 0.707, -0.707, 0])

        # Subscriber
        pose_sub = rospy.Subscriber('/Odometry', Odometry, self.pose_callback)
        usb_cam_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_callback)

        # Publisher 
        self.camera_pub = rospy.Publisher('/aruco_det/vis', Image, queue_size=1)
        self.det_vis_pub = rospy.Publisher('/aruco_det/det_vis', Image, queue_size=1)
        self.detect_state_pub = rospy.Publisher('/aruco_det/state', Bool, queue_size=1)
        self.detect_result = rospy.Publisher('/aruco_det/target_loc', PoseStamped, queue_size=1)
    
    def pose_callback(self, msg):
        self.state = msg
    
    def quat_mul(self, q1:np.array, q2:np.array):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        q = np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])
        return q

    def quat_rot_vector(self, q:np.array, v:np.array):
        q = q / np.linalg.norm(q)
        q_inv = np.array([q[0], -q[1], -q[2], -q[3]])
        qv = self.quat_mul(self.quat_mul(q, np.concatenate([[0], v])), q_inv)
        return qv[1 : ]
    
    def camera_callback(self, msg):
        self.num += 1
        # cv2.imwrite('/home/nuc_12/img/img_'+str(self.num)+'.jpg', msg)
        msg = self.bridge.imgmsg_to_cv2(msg)
        time_received = rospy.Time.now()
        # vis_msg = self.bridge.cv2_to_imgmsg(msg)
        # self.camera_pub.publish(vis_msg)
        now_state = self.state
        cv_image = msg
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.arucoDict,
            parameters=self.arucoParams)
        
        # corners = [np.array([[[ 835.,  243.],
        #                     [1070.,  226.],
        #                     [1086.,  464.],
        #                     [ 852.,  481.]]], dtype='float32')]

        flag = Bool()

        if len(corners) > 0:
            print('ids: ', ids)
            print("corners: ", corners)
            if ids[0][0] == 19:
                length = 0.2
            else:
                length = 0.025
            self.detect_state_pub.publish(True)
            draw_det_marker_img = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            # tmp = np.array([[-960, -540],[-960, -540],[-960, -540],[-960, -540]])

            # rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners+tmp, length, self.camera_Matrix,np.array([]))
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, length, self.camera_Matrix, self.distortion_Matrix)
            print("rvex.shape ", rvec.shape[0])
            for i in range(rvec.shape[0]):
                draw_det_marker_img = cv2.drawFrameAxes(draw_det_marker_img, self.camera_Matrix, self.distortion_Matrix,
                                                              rvec[i, :, :], tvec[i, :, :], 0.03)
                # draw_det_marker_img = cv2.aruco.drawAxis(draw_det_marker_img, self.camera_Matrix, self.distortion_Matrix,
                #                                          rvec[i, :, :], tvec[i, :, :], 0.03)
                self.det_vis_pub.publish(self.bridge.cv2_to_imgmsg(draw_det_marker_img))
                
                target_loc = PoseStamped()
                target_loc.header.stamp = time_received
                
                # target_loc.pose.position.x = tvec[0][0][0]
                # target_loc.pose.position.y = tvec[0][0][1]
                # target_loc.pose.position.z = tvec[0][0][2]

                pos_camera = np.array([tvec[0][0][0],
                                       tvec[0][0][1],
                                       tvec[0][0][2]])
                # print(ids[0][0])
                # if ids[0][0] == 43:
                print(pos_camera)
                target_loc.pose.position.x, target_loc.pose.position.y, target_loc.pose.position.z = \
                    self.quat_rot_vector(self.q_camera, pos_camera)
                print(target_loc)

                # target_loc.pose.orientation.x = atan(tvec[0][0][0] / tvec[0][0][2])
                # target_loc.pose.orientation.y = atan(tvec[0][0][1] / tvec[0][0][2])

                self.detect_result.publish(target_loc)

                flag.data = True
                self.detect_state_pub.publish(flag)

                break
            
        else:
            flag.data = False
            self.detect_state_pub.publish(flag)


if __name__ == "__main__":

    main_algorithm = Algorithm()
    rate = rospy.Rate(10)
    # cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        # retval, frame = cap.read()
        # # main_algorithm.camera_callback(frame)
        # if retval:
        #     main_algorithm.camera_callback(frame)
        # if cv2.waitKey(5) >= 0:
        #     break
        rate.sleep()