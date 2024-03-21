#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_generator/Trajectory.h"

void chatterCalllback(const trajectory_generator::Trajectory &msg)
{
	ROS_INFO("Header/\n");
	ROS_INFO("	seq [%d]\n", msg.header.seq);
	ROS_INFO("	frame_id [%s]\n", msg.header.frame_id.c_str() );
	ROS_INFO("	name [%s]\n", msg.name.c_str() );
	ROS_INFO("	sex [%d]\n", msg.sex);
	ROS_INFO("	age [%d]\n", msg.age);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe( "chatter", 1000, chatterCalllback );
	ros::spin();
	return 0;
}
