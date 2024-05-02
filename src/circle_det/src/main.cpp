// 定义一个节点，用于从点云中检测圆环
// 输入：点云
// 输出：圆环的位置

#include <ros/ros.h>
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

ros::Publisher pub, point_all_pub;
pcl::PCLPointCloud2 cloud_all;
std::vector<Eigen::Vector3d>circles;
int circle_num = 1;
bool finish_job = false;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // pcl::concatenatePointCloud(*cloud_msg, cloud_msg);
    if (finish_job)
        return;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);

    pcl::concatenatePointCloud(cloud_all, pcl_pc2, cloud_all);

    std::cout<<"all points: "<<cloud_all.data.size()<<std::endl;

    sensor_msgs::PointCloud2 cloud_all_msg;
    pcl_conversions::fromPCL(cloud_all, cloud_all_msg);
    point_all_pub.publish(cloud_all_msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // 下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_all, *cloud_all_filtered);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_all_filtered);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_filtered);

    // 发布下采样后的点云
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
    point_all_pub.publish(cloud_filtered_msg);

    // 聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);
	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.1);
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(150);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(clusterIndices);

	int j = 0;
	pcl::PCDWriter writer;
    std::cout<<"num: "<<clusterIndices.size()<<std::endl;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();it!=clusterIndices.end();++it)
	{
        bool circle_filter = false;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin();pit != it->indices.end();++pit)
		{
            // std::cout<<cloud_filtered->points[*pit]<<std::endl;
            if (cloud_filtered->points[*pit].data[0]>2 || cloud_filtered->points[*pit].data[0]<1 ||
                cloud_filtered->points[*pit].data[1]>1 || cloud_filtered->points[*pit].data[1]<-1)
                circle_filter = false;
			cloudCluster->points.push_back(cloud_filtered->points[*pit]);
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
        
        // 过滤
        if (center[0]>2 || center[0]<1 || center[1]>1 || center[1]<-1)
            continue;

        circles.push_back(center);
        // pub.publish(circles);
        if (circles.size() == circle_num){
            finish_job = true;
            return;
        }
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_det_node");
    ros::NodeHandle nh;

    // pub = nh.advertise<std::vector<Eigen::Vector3d>("/circle", 1);
    point_all_pub = nh.advertise<sensor_msgs::PointCloud2>("/all_points", 1);

    ros::Subscriber sub = nh.subscribe("/cloud_registered", 1, cloud_cb);

    ros::spin();
    return 0;
}
