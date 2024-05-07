// 定义一个节点，用于从点云中检测圆环
// 输入：点云
// 输出：圆环的位置

#include <ros/ros.h>
#include <cmath>
#include <iostream>

// #include <eigen/eigen>
#include<eigen3/Eigen/Core>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geometry_msgs/PoseStamped.h>

#include "circle_det/circles.h"

ros::Publisher pub, point_all_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_points(new pcl::PointCloud<pcl::PointXYZ>);
int circle_num = 1;
bool finish_job = false;

circle_det::circles circles_msg;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_single_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud_single_points);
    pcl::VoxelGrid<pcl::PointXYZ> vg_single_points;
    vg_single_points.setInputCloud(cloud_single_points);
    vg_single_points.setLeafSize(0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sinlge_points_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg_single_points.filter(*cloud_sinlge_points_filtered);
    *cloud_all_points += *cloud_sinlge_points_filtered;
    //
    pcl::VoxelGrid<pcl::PointXYZ> vg_single_points_tmp;
    vg_single_points_tmp.setInputCloud(cloud_all_points);
    vg_single_points_tmp.setLeafSize(0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_points_tmp(new pcl::PointCloud<pcl::PointXYZ>);
    vg_single_points_tmp.filter(*cloud_all_points_tmp);
    *cloud_all_points = *cloud_all_points_tmp;
    //
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_all(new pcl::search::KdTree<pcl::PointXYZ>);
	tree_all->setInputCloud(cloud_all_points);
	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.1);
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(150);
	ec.setSearchMethod(tree_all);
	ec.setInputCloud(cloud_all_points);
	ec.extract(clusterIndices);


    // 发布下采样后的点云
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(*cloud_all_points, cloud_filtered_msg);
    cloud_filtered_msg.header.frame_id = "camera_init";
    point_all_pub.publish(cloud_filtered_msg);

	for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();it!=clusterIndices.end();++it)
	{
        bool circle_filter = true;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin();pit != it->indices.end();++pit)
		{
            // if (cloud_all_points->points[*pit].data[0]>2 || cloud_all_points->points[*pit].data[0]<1 ||
            //     cloud_all_points->points[*pit].data[1]>1 || cloud_all_points->points[*pit].data[1]<-1)
            //     circle_filter = false;
			cloudCluster->points.push_back(cloud_all_points->points[*pit]);
		}
        if (circle_filter == false)
            continue;
        
        // 计算中心点
        Eigen::Vector3d center;
        center.setZero();
        for (int i = 0; i < cloudCluster->points.size(); i++)
        {
            center[0] += cloudCluster->points[i].data[0];
            center[1] += cloudCluster->points[i].data[1];
            center[2] += cloudCluster->points[i].data[2];
        }
        center /= cloudCluster->points.size();

        // 初始化圆拟合对象
        int N = cloudCluster->points.size();

        double sumX = 0.0; 
        double sumY = 0.0;
        double sumX2 = 0.0;
        double sumY2 = 0.0;
        double sumX3 = 0.0;
        double sumY3 = 0.0;
        double sumXY = 0.0;
        double sumXY2 = 0.0;
        double sumX2Y = 0.0;

        for (int pId = 0; pId < N; ++pId) {
            sumX += cloudCluster->points[pId].z;
            sumY += cloudCluster->points[pId].y;

            double x2 = cloudCluster->points[pId].z * cloudCluster->points[pId].z;
            double y2 = cloudCluster->points[pId].y * cloudCluster->points[pId].y;
            sumX2 += x2;
            sumY2 += y2;

            sumX3 += x2 * cloudCluster->points[pId].z;
            sumY3 += y2 * cloudCluster->points[pId].y;
            sumXY += cloudCluster->points[pId].z * cloudCluster->points[pId].y;
            sumXY2 += cloudCluster->points[pId].z * y2;
            sumX2Y += x2 * cloudCluster->points[pId].y;
        }

        double C, D, E, G, H;
        double a, b, c;

        C = N * sumX2 - sumX * sumX;
        D = N * sumXY - sumX * sumY;
        E = N * sumX3 + N * sumXY2 - (sumX2 + sumY2) * sumX;
        G = N * sumY2 - sumY * sumY;
        H = N * sumX2Y + N * sumY3 - (sumX2 + sumY2) * sumY;

        a = (H * D - E * G) / (C * G - D * D);
        b = (H * C - E * D) / (D * D - G * C);
        c = -(a * sumX + b * sumY + sumX2 + sumY2) / N;


        float radius = sqrt(a * a + b * b - 4 * c) / 2.0;
        // 过滤
        geometry_msgs::Point circle_center;
        circle_center.x = center[0];
        circle_center.y = -b/2.0;
        circle_center.z = -a/2.0;

        double err = 0.0;
        double e;
        double r2 = radius * radius;
        for (int pId = 0; pId < N; ++pId){
            e = (cloudCluster->points[pId].z - circle_center.z)*(cloudCluster->points[pId].z - circle_center.z) \
                +(cloudCluster->points[pId].y - circle_center.y)*(cloudCluster->points[pId].y - circle_center.y)  - r2;
            if (e > err) {
                err = e;
            }
        }
        if (radius > 0.8 || radius < 0.3 || err/N > 0.003 || circle_center.z < 1 || circle_center.z > 2)
            continue;
        std::cout<<err/N<<std::endl;
        std::cout<<"x: "<<center[0]<<" z: "<<-a/2.0<<" y: "<<-b/2.0<<" radius: "<<sqrt(a * a + b * b - 4 * c) / 2.0<<std::endl;

        if (circles_msg.pos.empty())
            circles_msg.pos.push_back(circle_center);
        else {
            bool is_in = false;
            for (auto it = circles_msg.pos.begin(); it != circles_msg.pos.end(); it++)
            {
                if (sqrt(pow(it->x - circle_center.x, 2) + pow(it->y - circle_center.y, 2) + pow(it->z - circle_center.z, 2)) < 0.5)
                {
                    is_in = true;
                }
            }
            if (!is_in)
                circles_msg.pos.push_back(circle_center);
        }
	}
    pub.publish(circles_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_det_node");
    ros::NodeHandle nh;

    pub = nh.advertise<circle_det::circles>("/circle", 1);
    point_all_pub = nh.advertise<sensor_msgs::PointCloud2>("/all_points", 1);

    ros::Subscriber sub = nh.subscribe("/cloud_registered", 1, cloud_cb);

    ros::spin();
    return 0;
}
