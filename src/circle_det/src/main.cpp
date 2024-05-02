// 定义一个节点，用于从点云中检测圆环
// 输入：点云
// 输出：圆环的位置

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub, point_all_pub;
pcl::PCLPointCloud2 cloud_all;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // pcl::concatenatePointCloud(*cloud_msg, cloud_msg);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);

    pcl::concatenatePointCloud(cloud_all, pcl_pc2, cloud_all);

    std::cout<<"all points: "<<cloud_all.data.size()<<std::endl;

    // sensor_msgs::PointCloud2 cloud_all_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all_msg(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl_conversions::fromPCL(cloud_all, cloud_all_msg);
    // point_all_pub.publish(cloud_all_msg);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // 下采样
    pcl::fromPCLPointCloud2(cloud_all, *cloud_all_msg);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_all_msg);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_filtered);

    // 发布下采样后的点云
    sensor_msgs::PointCloud2 cloud_filtered_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_filtered_msg);
    point_all_pub.publish(cloud_filtered_msg);

    // // 圆环检测
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold(0.01);
    // seg.setInputCloud(cloud_filtered);
    // seg.segment(*inliers, *coefficients);

    // // 提取圆环
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(cloud_filtered);
    // extract.setIndices(inliers);
    // extract.setNegative(false);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circle(new pcl::PointCloud<pcl::PointXYZ>);
    // extract.filter(*cloud_circle);

    // // 发布圆环
    // sensor_msgs::PointCloud2 cloud_circle_msg;
    // pcl::toROSMsg(*cloud_circle, cloud_circle_msg);
    // pub.publish(cloud_circle_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_det_node");
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("/circle", 1);
    point_all_pub = nh.advertise<sensor_msgs::PointCloud2>("/all_points", 1);

    ros::Subscriber sub = nh.subscribe("/cloud_registered", 1, cloud_cb);

    ros::spin();
    return 0;
}
