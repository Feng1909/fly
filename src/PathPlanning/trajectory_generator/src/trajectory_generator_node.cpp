#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

#include "trajectory_generator/Trajectory.h"

// Useful customized headers
#include "trajectory_generator/trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

// Param from launch file
    double _vis_traj_width;
    double _limit_vel, _limit_acc;
    double _limit_d;  // allowable width of the trajectory
    double _limit_jerk;
    double _dist_thresh;

// ros related
    ros::Subscriber _way_pts_sub, _traj_msg_sub, _detect_pose_sub;
    ros::Publisher _wp_path_vis_pub, _wp_traj_vis_pub;
    ros::Publisher _traj_pub;

    bool _reach_point = false;

    // Ros msg
    trajectory_generator::Trajectory _traj_msg;

// for planning
    MatrixXd _coord;
    VectorXd _polyTime;

// declare
    void visWayPointPath(MatrixXd path);
    void vis_traj_point_path(MatrixXd path);
    VectorXd timeAllocation( MatrixXd Path);
    void trajGeneration(Eigen::MatrixXd path);
    void rcvWaypointsCallBack(const nav_msgs::Path & wp);
    void send_traj_msg(Eigen::MatrixXd path, Eigen::VectorXd time);
    void rcv_point(const nav_msgs::Odometry &msg);

    // test msg
    void listen_traj_msg(const trajectory_generator::Trajectory &msg);

void rcv_point(const nav_msgs::Odometry &msg) {
    double px, py, pz;
    px = msg.pose.pose.position.x;
    py = msg.pose.pose.position.y;
    pz = msg.pose.pose.position.z;
    if (px*px + py*py + (pz-1.0)*(pz-1.0) < _dist_thresh && !_reach_point) _reach_point = true;
}

void listen_traj_msg(const trajectory_generator::Trajectory &msg) {
    geometry_msgs::Point p;
    for (int i=0; i<msg.pos.size(); ++i) {
        p = msg.pos[i];
        ROS_INFO("x: %f, y: %f, z: %f", p.x, p.y, p.z);
    }
    
}

//Get the path points 
void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }

    // Eigen::MatrixXd waypoints(wp_list.size(), 3);
    // for (int k = 0; k < (int)wp_list.size(); k++) {
    //     waypoints.row(k) = wp_list[k].transpose();
    // }
    // Eigen::MatrixXd waypoints(2, 3);
    // if (!_reach_point) {
    //     waypoints << 0, 0, 0,
    //                  0, 0, 1;
    // }
    // else {
    //     waypoints << 0, 0, 1,
    //                  1, 0, 1;
    // }
    Eigen::MatrixXd waypoints(3, 3);
    waypoints << 0.0, 0.0, 0.0,
                 0.0, 0.0, 1.0,
                 1.0, 0.0, 1.0;

    // generate trajectory based on the waypoints
    trajGeneration(waypoints);
}

void trajGeneration(Eigen::MatrixXd path)
{
    TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    _polyTime  = timeAllocation(path);

    // generate a minimum-snap piecewise monomial polynomial-based trajectory
    _coord = trajectoryGeneratorWaypoint.CoordGeneration(path, _polyTime, _limit_d, _limit_vel, _limit_acc, _limit_jerk);

    // visWayPointPath(path);
    visWayPointPath(path);

    // show the optimized trajectory
    vis_traj_point_path(_coord);
    // std::cout << std::endl << _coord << std::endl;

    // send the trajectory to the controller
    send_traj_msg(_coord, _polyTime);
}

void send_traj_msg(Eigen::MatrixXd path, Eigen::VectorXd time) {
    int path_len = path.rows();    
    _traj_msg.header.stamp = ros::Time::now();
    _traj_msg.header.frame_id = "map";
    
    Eigen::VectorXd time_each_point(path_len);
    // time
    double time_sum = time.sum();
    for (int i=0; i<path_len; ++i) {
        time_each_point(i) = time_sum / double(path_len) * double(i);
        _traj_msg.time.push_back(time_each_point(i));
        _traj_msg.yaw.push_back(0.0);
    }
    geometry_msgs::Point p;
    for (int i=0; i<path_len; ++i) {
        p.x = path(i, 0);
        p.y = path(i, 1);
        p.z = path(i, 2);
        _traj_msg.pos.push_back(p);
    }

    // _traj_pub.publish(_traj_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/limit_vel", _limit_vel, 1.0);
    nh.param("planning/limit_acc", _limit_acc, 1.0);
    nh.param("planning/limit_d", _limit_d, 1.0);
    nh.param("planning/limit_jerk", _limit_jerk, 1.0);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
    nh.param("dist_thresh", _dist_thresh, 0.1);
    
    _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack);
    _detect_pose_sub = nh.subscribe("/mavros/local_position/odom", 1, rcv_point);
    

    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

    _traj_pub = nh.advertise<trajectory_generator::Trajectory>("traj_msg", 1);

    // test
    _traj_msg_sub = nh.subscribe("traj_msg", 1, &listen_traj_msg);

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();
        status = ros::ok();  
        _traj_pub.publish(_traj_msg);
        rate.sleep();
    }
    return 0;
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;
    
    line_list.color.r = 1.0;
    line_list.color.g = 0.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

void vis_traj_point_path(MatrixXd path) {
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;
    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    _wp_traj_vis_pub.publish(points);
    _wp_traj_vis_pub.publish(line_list);
}

VectorXd timeAllocation( MatrixXd Path)
{ 
    VectorXd time(Path.rows() - 1);
    int idx = time.rows();
    double tmp_x, tmp_y, tmp_z;
    tmp_x = abs(Path(0, 0) - Path(1, 0));
    tmp_y = abs(Path(0, 1) - Path(1, 1));
    tmp_z = abs(Path(0, 2) - Path(1, 2));
    time(0) = 5.0 * max(tmp_x, max(tmp_y, tmp_z));
    for (int i=1; i<idx-1; ++i) {
        tmp_x = abs(Path(i, 0) - Path(i+1, 0));
        tmp_y = abs(Path(i, 1) - Path(i+1, 1));
        tmp_z = abs(Path(i, 2) - Path(i+1, 2));
        time(i) = 2.5 * max(tmp_x, max(tmp_y, tmp_z));
    }
    tmp_x = abs(Path(idx, 0) - Path(idx-1, 0));
    tmp_y = abs(Path(idx, 1) - Path(idx-1, 1));
    tmp_z = abs(Path(idx, 2) - Path(idx-1, 2));
    time(idx-1) = 5.0 * max(tmp_x, max(tmp_y, tmp_z));    
    return time;
}