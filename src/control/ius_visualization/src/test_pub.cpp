#include <ros/ros.h>
#include <ius_msgs/PillarPos.h>
#include <ius_msgs/LoopPos.h>

ros::Publisher loop_pos_pub;
ros::Publisher pillar_pos_pub;

void topic_publish() {
  ius_msgs::LoopPos loop_pos_msg;
  loop_pos_msg.header.frame_id = "world";
  loop_pos_msg.header.stamp = ros::Time::now();
  loop_pos_msg.loop_id = 1;
  loop_pos_msg.loop_pos.x = 1;
  loop_pos_msg.loop_pos.y = 1;
  loop_pos_msg.loop_pos.z = 1;
  loop_pos_pub.publish(loop_pos_msg);
  loop_pos_msg.header.frame_id = "world";
  loop_pos_msg.header.stamp = ros::Time::now();
  loop_pos_msg.loop_id = 2;
  loop_pos_msg.loop_pos.x = 2;
  loop_pos_msg.loop_pos.y = 1;
  loop_pos_msg.loop_pos.z = 1;
  loop_pos_pub.publish(loop_pos_msg);

  ius_msgs::PillarPos pillar_pos_msg;
  pillar_pos_msg.header.frame_id = "world";
  pillar_pos_msg.header.stamp = ros::Time::now();
  pillar_pos_msg.pillar_id = 1;
  pillar_pos_msg.pillar_pos.x = 2;
  pillar_pos_msg.pillar_pos.y = 2;
  pillar_pos_msg.pillar_pos.z = 1;
  pillar_pos_pub.publish(pillar_pos_msg);
  pillar_pos_msg.header.frame_id = "world";
  pillar_pos_msg.header.stamp = ros::Time::now();
  pillar_pos_msg.pillar_id = 2;
  pillar_pos_msg.pillar_pos.x = 3;
  pillar_pos_msg.pillar_pos.y = 2;
  pillar_pos_msg.pillar_pos.z = 1;
  pillar_pos_pub.publish(pillar_pos_msg);
  pillar_pos_msg.header.frame_id = "world";
  pillar_pos_msg.header.stamp = ros::Time::now();
  pillar_pos_msg.pillar_id = 3;
  pillar_pos_msg.pillar_pos.x = 4;
  pillar_pos_msg.pillar_pos.y = 2;
  pillar_pos_msg.pillar_pos.z = 1;
  pillar_pos_pub.publish(pillar_pos_msg);
  pillar_pos_msg.header.frame_id = "world";
  pillar_pos_msg.header.stamp = ros::Time::now();
  pillar_pos_msg.pillar_id = 4;
  pillar_pos_msg.pillar_pos.x = 5;
  pillar_pos_msg.pillar_pos.y = 2;
  pillar_pos_msg.pillar_pos.z = 1;
  pillar_pos_pub.publish(pillar_pos_msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_pub");
  ros::NodeHandle nh;

  loop_pos_pub = nh.advertise<ius_msgs::LoopPos>("/ius_uav/loop_pos", 50);
  pillar_pos_pub = nh.advertise<ius_msgs::PillarPos>("/ius_uav/pillar_pos", 50);

  ros::Rate rate(50);
  while(ros::ok()) {
    topic_publish();
    rate.sleep();
  }

  return 0;
}
