#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <ius_msgs/PillarPos.h>
#include <ius_msgs/LoopPos.h>
#include <ius_msgs/Trajectory.h>
#include <ius_msgs/TunnelPos.h>
#include <ius_msgs/MazePos.h>

ros::Publisher v_pillar_pub;
ros::Publisher v_loop_pub;
ros::Publisher v_trajectory_pub;
ros::Publisher v_tunnel_pub;
ros::Publisher v_maze_pub;

void pillar_pos_cb(const ius_msgs::PillarPos::ConstPtr& msg) {
  visualization_msgs::Marker v_pillar_msg;
  v_pillar_msg.header.frame_id = "world";
  v_pillar_msg.header.stamp = ros::Time::now();

  v_pillar_msg.type = visualization_msgs::Marker::CYLINDER;
  v_pillar_msg.action = visualization_msgs::Marker::MODIFY;
  v_pillar_msg.scale.x = 0.3;
  v_pillar_msg.scale.y = 0.3;
  v_pillar_msg.scale.z = 2.0;
  v_pillar_msg.color.r = 1.0;
  v_pillar_msg.color.g = 0.0;
  v_pillar_msg.color.b = 1.0;
  v_pillar_msg.color.a = 0.4;

  v_pillar_msg.id = msg->pillar_id;
  v_pillar_msg.pose.orientation.w = 1.0;
  v_pillar_msg.pose.orientation.x = 0.0;
  v_pillar_msg.pose.orientation.y = 0.0;
  v_pillar_msg.pose.orientation.z = 0.0;
  v_pillar_msg.pose.position = msg->pillar_pos;

  v_pillar_pub.publish(v_pillar_msg);
}

void loop_pos_cb(const ius_msgs::LoopPos::ConstPtr& msg) {
  visualization_msgs::Marker v_loop_msg;
  v_loop_msg.header.frame_id = "world";
  v_loop_msg.header.stamp = ros::Time::now();

  v_loop_msg.type = visualization_msgs::Marker::CYLINDER;
  v_loop_msg.action = visualization_msgs::Marker::MODIFY;
  v_loop_msg.scale.x = 1.5;
  v_loop_msg.scale.y = 1.5;
  v_loop_msg.scale.z = 0.05;
  v_loop_msg.color.r = 1.0;
  v_loop_msg.color.g = 1.0;
  v_loop_msg.color.b = 1.0;
  v_loop_msg.color.a = 0.7;

  v_loop_msg.id = msg->loop_id;
  v_loop_msg.pose.orientation.w = 0.7072;
  v_loop_msg.pose.orientation.x = 0.0;
  v_loop_msg.pose.orientation.y = 0.7072;
  v_loop_msg.pose.orientation.z = 0.0;
  v_loop_msg.pose.position = msg->loop_pos;
  v_loop_pub.publish(v_loop_msg);
}

void trajectory_cb(const ius_msgs::Trajectory::ConstPtr& msg) {
  nav_msgs::Path v_trajectory_msg;
  v_trajectory_msg.header.frame_id = "world";
  v_trajectory_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < msg->pos.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position = msg->pos[i];
    double yaw = msg->yaw[i];
    pose.pose.orientation.w = cos(yaw/2);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = sin(yaw/2);
    v_trajectory_msg.poses.push_back(pose);
  }
  v_trajectory_pub.publish(v_trajectory_msg);
}

void tunnel_pos_cb(const ius_msgs::TunnelPos::ConstPtr& msg) {
  visualization_msgs::Marker v_tunnel_msg;
  v_tunnel_msg.header.frame_id = "world";
  v_tunnel_msg.header.stamp = ros::Time::now();

  v_tunnel_msg.type = visualization_msgs::Marker::POINTS;
  v_tunnel_msg.action = visualization_msgs::Marker::MODIFY;
  v_tunnel_msg.scale.x = 0.3;
  v_tunnel_msg.scale.y = 0.3;
  v_tunnel_msg.scale.z = 0.3;
  v_tunnel_msg.color.r = 1.0;
  v_tunnel_msg.color.g = 0.0;
  v_tunnel_msg.color.b = 0.0;
  v_tunnel_msg.color.a = 0.4;
  v_tunnel_msg.colors.push_back(v_tunnel_msg.color);
  v_tunnel_msg.color.r = 0.0;
  v_tunnel_msg.color.g = 1.0;
  v_tunnel_msg.color.b = 0.0;
  v_tunnel_msg.color.a = 0.4;
  v_tunnel_msg.colors.push_back(v_tunnel_msg.color);

  v_tunnel_msg.points.push_back(msg->tunnel_pos_in);
  v_tunnel_msg.points.push_back(msg->tunnel_pos_out);
  // v_tunnel_msg.id = 1;
  // v_tunnel_msg.pose.orientation.w = 1.0;
  // v_tunnel_msg.pose.orientation.x = 0.0;
  // v_tunnel_msg.pose.orientation.y = 0.0;
  // v_tunnel_msg.pose.orientation.z = 0.0;
  // v_tunnel_msg.pose.position = msg->tunnel_pos_in;
  v_tunnel_pub.publish(v_tunnel_msg);

  // v_tunnel_msg.id = 2;
  // v_tunnel_msg.pose.position = msg->tunnel_pos_out;
  // v_tunnel_pub.publish(v_tunnel_msg);
}

void maze_pos_cb(const ius_msgs::MazePos::ConstPtr& msg) {
  visualization_msgs::Marker v_maze_msg;
  v_maze_msg.header.frame_id = "world";
  v_maze_msg.header.stamp = ros::Time::now();

  v_maze_msg.type = visualization_msgs::Marker::POINTS;
  v_maze_msg.action = visualization_msgs::Marker::MODIFY;
  v_maze_msg.scale.x = 0.3;
  v_maze_msg.scale.y = 0.3;
  v_maze_msg.scale.z = 2.0;
  v_maze_msg.color.r = 1.0;
  v_maze_msg.color.g = 0.0;
  v_maze_msg.color.b = 0.0;
  v_maze_msg.color.a = 0.4;
  v_maze_msg.colors.push_back(v_maze_msg.color);
  v_maze_msg.color.r = 0.0;
  v_maze_msg.color.g = 1.0;
  v_maze_msg.color.b = 0.0;
  v_maze_msg.color.a = 0.4;
  v_maze_msg.colors.push_back(v_maze_msg.color);
  v_maze_msg.color.r = 0.0;
  v_maze_msg.color.g = 0.0;
  v_maze_msg.color.b = 1.0;
  v_maze_msg.color.a = 0.4;
  v_maze_msg.colors.push_back(v_maze_msg.color);

  v_maze_msg.points.push_back(msg->maze_pos_in);
  v_maze_msg.points.push_back(msg->maze_pos_mid);
  v_maze_msg.points.push_back(msg->maze_pos_out);

  v_maze_pub.publish(v_maze_msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ius_vis");
  ros::NodeHandle nh;

  v_pillar_pub = nh.advertise<visualization_msgs::Marker>
                           ("/ius_uav/visualization/pillar", 10);
  v_loop_pub = nh.advertise<visualization_msgs::Marker>
                           ("/ius_uav/visualization/loop", 10);
  v_trajectory_pub = nh.advertise<nav_msgs::Path>
                           ("/ius_uav/visualization/trajectory", 10);
  v_tunnel_pub = nh.advertise<visualization_msgs::Marker>
                           ("/ius_uav/visualization/tunnel", 10);
  v_maze_pub = nh.advertise<visualization_msgs::Marker>
                           ("/ius_uav/visualization/maze", 10);
  
  ros::Subscriber loop_pos_sub = nh.subscribe<ius_msgs::LoopPos>
                                 ("/ius_uav/loop_pos", 10,
                                  loop_pos_cb);
  ros::Subscriber pillar_sub = nh.subscribe<ius_msgs::PillarPos>
                               ("/ius_uav/pillar_pos", 10,
                                pillar_pos_cb);
  ros::Subscriber trajectory_sub = nh.subscribe<ius_msgs::Trajectory>
                                   ("/ius_uav/trajectory", 1,
                                    trajectory_cb);
  ros::Subscriber tunnel_sub = nh.subscribe<ius_msgs::TunnelPos>
                                ("/ius_uav/tunnel_pos", 10,
                                  tunnel_pos_cb);
  ros::Subscriber maze_sub = nh.subscribe<ius_msgs::MazePos>
                              ("/ius_uav/maze_pos", 10,
                                maze_pos_cb);
  ros::spin();
  return 0;
}
