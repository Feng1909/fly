#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from collections import deque

class DroneNavigator:
    def __init__(self):
        rospy.init_node("drone_frame_navigator")

        self.point_clouds = deque()  # 用于维护点云数据的队列
        self.odom = None

        # 订阅点云数据和里程计数据
        rospy.Subscriber("/cloud_registered", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber("~odom", Odometry, self.odom_callback, queue_size=1, tcp_nodelay=True)

        # 发布目标位置和过滤后的点云
        self.target_pub = rospy.Publisher("/center_position", PoseStamped, queue_size=1)
        self.filtered_cloud_pub = rospy.Publisher("/filtered_cloud", PointCloud2, queue_size=1)

    def point_cloud_callback(self, msg):
        try:
            # 检查队列长度是否超过5，如果超过则删除最老的数据
            if len(self.point_clouds) >= 5:
                self.point_clouds.popleft()
            self.point_clouds.append(msg)  # 将新的点云数据添加到队列末尾
            rospy.loginfo("Received PointCloud2 message")
        except Exception as e:
            rospy.logerr(f"Error in point_cloud_callback: {e}")

    def odom_callback(self, msg):
        try:
            self.odom = msg
            rospy.loginfo("Received Odometry message")
        except Exception as e:
            rospy.logerr(f"Error in odom_callback: {e}")

    def navigate_drone_to_box(self):
        try:
            if not self.point_clouds or self.odom is None:
                return None, None

            # 提取位姿信息
            position = self.odom.pose.pose.position

            # 获取x方向上的坐标范围
            x_min = position.x + 0.2  # 无人机前方0.2m的最小x值
            x_max = position.x + 1.0  # 无人机前方1m的最大x值

            # 获取y和z方向上的坐标范围
            y_min, y_max = -1.5, 1.5  # 世界坐标系下y轴的范围
            z_min, z_max = 1.0, 2.0   # 世界坐标系下z轴的范围

            # 合并所有点云数据
            merged_points = []
            for point_cloud in list(self.point_clouds):  # 使用副本进行迭代
                points = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)
                merged_points.extend(points)

            # 过滤点，仅保留指定范围内的点
            filtered_points = [point for point in merged_points if
                               x_min < point[0] < x_max and
                               y_min < point[1] < y_max and
                               z_min < point[2] < z_max]

            if not filtered_points:
                rospy.logwarn("No points found in the specified range.")
                return None, None

            # 计算x和y的最大值和最小值的平均值
            x_values = [point[0] for point in filtered_points]
            y_values = [point[1] for point in filtered_points]
            x_center = (min(x_values) + max(x_values)) / 2.0
            y_center = (min(y_values) + max(y_values)) / 2.0

            z_max_value = max(point[2] for point in filtered_points)

            # 计算目标位置（取z轴的最大值，并向下偏移0.3m）
            target_position = [x_center, y_center, z_max_value - 0.2]

            return target_position, filtered_points
        except Exception as e:
            rospy.logerr(f"Error in navigate_drone_to_box: {e}")
            return None, None

    def publish_filtered_cloud(self, filtered_points):
        try:
            # 创建PointCloud2消息
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "world"

            # 定义点云数据的字段
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
            ]

            # 创建PointCloud2消息
            filtered_cloud = pc2.create_cloud(header, fields, filtered_points)
            self.filtered_cloud_pub.publish(filtered_cloud)
            rospy.loginfo("Published filtered point cloud")
        except Exception as e:
            rospy.logerr(f"Error in publish_filtered_cloud: {e}")

    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            # 计算目标位置和过滤后的点云
            target_position, filtered_points = self.navigate_drone_to_box()
            
            if target_position is not None:
                try:
                    # 发布目标位置
                    target_pose = PoseStamped()
                    target_pose.header.stamp = rospy.Time.now()
                    target_pose.header.frame_id = "world"  # 发布到世界坐标系中
                    
                    target_pose.pose.position.x = target_position[0]
                    target_pose.pose.position.y = target_position[1]
                    target_pose.pose.position.z = target_position[2]
                    target_pose.pose.orientation.w = 1.0
                    
                    self.target_pub.publish(target_pose)
                    rospy.loginfo("Published target position")
                except Exception as e:
                    rospy.logerr(f"Error in publishing target position: {e}")

            if filtered_points:
                self.publish_filtered_cloud(filtered_points)

            rate.sleep()

if __name__ == "__main__":
    try:
        navigator = DroneNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
