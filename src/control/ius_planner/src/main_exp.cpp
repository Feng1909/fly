// #include <plan_env/grid_map.h>

#include "ius_planner/planner.h"
// #include <ius_msgs/PillarPos.h>
// #include <ius_msgs/TunnelPos.h>
// #include <ius_msgs/LoopPos.h>
// #include <ius_msgs/MazePos.h>
#include "ius_planner/pos_param.h"

// GridMap::Ptr grid_map_;

// planning visualization
ego_planner::PlanningVisualization::Ptr visualization_;

// topic define
ros::Publisher traj_pub_;
ros::Subscriber local_pos_sub;
ros::Subscriber pillar_sub, tunnel_sub,maze_sub,loop_sub,m_loop_sub;
void param_init();
// trajectory
PolynomialTraj traj_;
ius_msgs::Trajectory traj_msg_;

Posparam pos_param_;
static geometry_msgs::PoseStamped aim;


geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    local_pos = *msg;
}

  void trajectory_update()
  {
       trajA_1 = PolynomialTraj::one_segment_traj_gen(start_pos_A, start_vel_A, start_acc_A,
                                                  mid_pos1_A, mid_vel1_A, mid_acc1_A, 3.0);
       trajA_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_A, mid_vel1_A, mid_acc1_A,
                                                        mid_pos2_A, mid_vel2_A, mid_acc2_A, 3.0);
      //  trajA_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_A, mid_vel2_A, mid_acc2_A,
      //                                                   mid_pos3_A, mid_vel3_A, mid_acc3_A, 3.0); 
      //  trajA_4 = PolynomialTraj::one_segment_traj_gen(mid_pos3_A, mid_vel3_A, mid_acc3_A,
      //                                                   mid_pos4_A, mid_vel4_A, mid_acc4_A, 3.0); 
      //  trajA_5 = PolynomialTraj::one_segment_traj_gen(mid_pos4_A, mid_vel4_A, mid_acc4_A,
      //                                                   end_pos_A, end_vel_A, end_acc_A, 3.0);                                                  
      //  trajB_1 = PolynomialTraj::one_segment_traj_gen(end_pos_A, end_vel_A, end_acc_A,
      //                                                   start_pos_B, start_vel_B, start_acc_B, 3.0);            
      //  trajB_2 = PolynomialTraj::one_segment_traj_gen(start_pos_B, start_vel_B, start_acc_B,
      //                                                   mid_pos1_B, mid_vel1_B, mid_acc1_B, 2.0);
      //  trajB_3 = PolynomialTraj::one_segment_traj_gen(mid_pos1_B, mid_vel1_B, mid_acc1_B,
      //                                                   end_pos_B, end_vel_B, end_acc_B, 3.0);

      //   trajC_0 = PolynomialTraj::one_segment_traj_gen(end_pos_B, end_vel_B, end_acc_B,
      //                                             start_pos_C, start_vel_C, start_acc_C, 3.0);

      //   trajC_1 = PolynomialTraj::one_segment_traj_gen(start_pos_C, start_vel_C, start_acc_C,
      //                                                     mid_pos1_C, mid_vel1_C, mid_acc1_C, 3.0);

      //   trajC_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_C, mid_vel1_C, mid_acc1_C,
      //                                                       mid_pos2_C, mid_vel2_C, mid_acc2_C, 3.0);

      //   trajC_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_C, mid_vel2_C, mid_acc2_C,
      //                                                       end_pos_C, end_vel_C, end_acc_C, 3.0);

      //   trajD_1 = PolynomialTraj::one_segment_traj_gen(end_pos_C, end_vel_C, end_acc_C,
      //                                                   start_pos_D, start_vel_D, start_acc_D, 3.0);
      //   trajD_2 = PolynomialTraj::one_segment_traj_gen(start_pos_D, start_vel_D, start_acc_D,
      //                                                   end_pos_D, end_vel_D, end_acc_D, 3.0);

      //   trajE_1 = PolynomialTraj::one_segment_traj_gen(end_pos_D, end_vel_D, end_acc_D,
      //                                                   start_pos_E, start_vel_E, start_acc_E, 3.0);
      //   trajE_2 = PolynomialTraj::one_segment_traj_gen(start_pos_E, start_vel_E, start_acc_E,
      //                                                   end_pos_E, end_vel_E, end_acc_E, 3.0);
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ius_planner_node");
  ros::NodeHandle nh("~");

  pos_param_.load_param(nh);
  
  param_init();

  trajectory_update();

  local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                                    ("/mavros/local_position/pose", 1, local_pos_cb);
  traj_pub_ = nh.advertise<ius_msgs::Trajectory>("trajectory", 1);

  visualization_.reset(new ego_planner::PlanningVisualization(nh));  


  ros::Rate loop_rate(10);
  while (ros::ok())
  {

    std::vector<Eigen::Vector3d> pos_sampled;
    // sample traj pos
    pos_sampled.clear();
    traj_msg_.pos.clear();
    traj_msg_.yaw.clear();
    traj_msg_.time.clear();

    // int flag_mission = check_mission(local_pos);
    trajectory_update();

    // clear traj_
    traj_.reset();

    /*  Task A traj init */                                      
    traj_.addSegment(trajA_1.getCoef(0)[0], trajA_1.getCoef(1)[0], trajA_1.getCoef(2)[0], trajA_1.getTimes()[0]);
    traj_.addSegment(trajA_2.getCoef(0)[0], trajA_2.getCoef(1)[0], trajA_2.getCoef(2)[0], trajA_2.getTimes()[0]);
    // traj_.addSegment(trajA_3.getCoef(0)[0], trajA_3.getCoef(1)[0], trajA_3.getCoef(2)[0], trajA_3.getTimes()[0]);
    // traj_.addSegment(trajA_4.getCoef(0)[0], trajA_4.getCoef(1)[0], trajA_4.getCoef(2)[0], trajA_4.getTimes()[0]);
    // traj_.addSegment(trajA_5.getCoef(0)[0], trajA_5.getCoef(1)[0], trajA_5.getCoef(2)[0], trajA_5.getTimes()[0]);

    // // /*  Task B traj init */ 
    // traj_.addSegment(trajB_1.getCoef(0)[0], trajB_1.getCoef(1)[0], trajB_1.getCoef(2)[0], trajB_1.getTimes()[0]);                                 
    // traj_.addSegment(trajB_2.getCoef(0)[0], trajB_2.getCoef(1)[0], trajB_2.getCoef(2)[0], trajB_2.getTimes()[0]);
    // traj_.addSegment(trajB_3.getCoef(0)[0], trajB_3.getCoef(1)[0], trajB_3.getCoef(2)[0], trajB_3.getTimes()[0]);

    // // /*  Task C traj init */
    // traj_.addSegment(trajC_0.getCoef(0)[0], trajC_0.getCoef(1)[0], trajC_0.getCoef(2)[0], trajC_0.getTimes()[0]);
    // traj_.addSegment(trajC_1.getCoef(0)[0], trajC_1.getCoef(1)[0], trajC_1.getCoef(2)[0], trajC_1.getTimes()[0]);                                 
    // traj_.addSegment(trajC_2.getCoef(0)[0], trajC_2.getCoef(1)[0], trajC_2.getCoef(2)[0], trajC_2.getTimes()[0]);
    // traj_.addSegment(trajC_3.getCoef(0)[0], trajC_3.getCoef(1)[0], trajC_3.getCoef(2)[0], trajC_3.getTimes()[0]);

    // /*  Task D traj init */ 
    // traj_.addSegment(trajD_1.getCoef(0)[0], trajD_1.getCoef(1)[0], trajD_1.getCoef(2)[0], trajD_1.getTimes()[0]);                                 
    // traj_.addSegment(trajD_2.getCoef(0)[0], trajD_2.getCoef(1)[0], trajD_2.getCoef(2)[0], trajD_2.getTimes()[0]);

    // /*  Task E traj init */
    // traj_.addSegment(trajE_1.getCoef(0)[0], trajE_1.getCoef(1)[0], trajE_1.getCoef(2)[0], trajE_1.getTimes()[0]);                                 
    // traj_.addSegment(trajE_2.getCoef(0)[0], trajE_2.getCoef(1)[0], trajE_2.getCoef(2)[0], trajE_2.getTimes()[0]);

    for (double t = 0.0; t < 4.6; t += 0.1)
    {
      const auto& pt = traj_.evaluate(t);
      pos_sampled.push_back(pt);

      traj_msg_.header.stamp = ros::Time::now();
      traj_msg_.header.frame_id = "world";
      geometry_msgs::Point point;
      point.x = pt.x();
      point.y = pt.y();
      point.z = pt.z();     
      traj_msg_.pos.push_back(point);

      traj_msg_.yaw.push_back(0.0);
      traj_msg_.time.push_back(t);
    }

    visualization_->displayGlobalPathList(pos_sampled, 0.1, 1);
    traj_pub_.publish(traj_msg_);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void param_init(){
  start_pos_A = Eigen::Vector3d(pos_param_.start_pos_vec_A[0], pos_param_.start_pos_vec_A[1], pos_param_.start_pos_vec_A[2]);
  start_vel_A = Eigen::Vector3d(pos_param_.start_vel_vec_A[0], pos_param_.start_vel_vec_A[1], pos_param_.start_vel_vec_A[2]);
  start_acc_A = Eigen::Vector3d(pos_param_.start_acc_vec_A[0], pos_param_.start_acc_vec_A[1], pos_param_.start_acc_vec_A[2]);
  mid_pos1_A = Eigen::Vector3d(pos_param_.mid_pos1_vec_A[0], pos_param_.mid_pos1_vec_A[1], pos_param_.mid_pos1_vec_A[2]);
  mid_vel1_A = Eigen::Vector3d(pos_param_.mid_vel1_vec_A[0], pos_param_.mid_vel1_vec_A[1], pos_param_.mid_vel1_vec_A[2]);
  mid_acc1_A = Eigen::Vector3d(pos_param_.mid_acc1_vec_A[0], pos_param_.mid_acc1_vec_A[1], pos_param_.mid_acc1_vec_A[2]);
  mid_pos2_A = Eigen::Vector3d(pos_param_.mid_pos2_vec_A[0], pos_param_.mid_pos2_vec_A[1], pos_param_.mid_pos2_vec_A[2]);
  mid_vel2_A = Eigen::Vector3d(pos_param_.mid_vel2_vec_A[0], pos_param_.mid_vel2_vec_A[1], pos_param_.mid_vel2_vec_A[2]);
  mid_acc2_A = Eigen::Vector3d(pos_param_.mid_acc2_vec_A[0], pos_param_.mid_acc2_vec_A[1], pos_param_.mid_acc2_vec_A[2]);
  mid_pos3_A = Eigen::Vector3d(pos_param_.mid_pos3_vec_A[0], pos_param_.mid_pos3_vec_A[1], pos_param_.mid_pos3_vec_A[2]);
  mid_vel3_A = Eigen::Vector3d(pos_param_.mid_vel3_vec_A[0], pos_param_.mid_vel3_vec_A[1], pos_param_.mid_vel3_vec_A[2]);
  mid_acc3_A = Eigen::Vector3d(pos_param_.mid_acc3_vec_A[0], pos_param_.mid_acc3_vec_A[1], pos_param_.mid_acc3_vec_A[2]);
  mid_pos4_A = Eigen::Vector3d(pos_param_.mid_pos4_vec_A[0], pos_param_.mid_pos4_vec_A[1], pos_param_.mid_pos4_vec_A[2]);
  mid_vel4_A = Eigen::Vector3d(pos_param_.mid_vel4_vec_A[0], pos_param_.mid_vel4_vec_A[1], pos_param_.mid_vel4_vec_A[2]);
  mid_acc4_A = Eigen::Vector3d(pos_param_.mid_acc4_vec_A[0], pos_param_.mid_acc4_vec_A[1], pos_param_.mid_acc4_vec_A[2]);
  end_pos_A = Eigen::Vector3d(pos_param_.end_pos_vec_A[0], pos_param_.end_pos_vec_A[1], pos_param_.end_pos_vec_A[2]);
  end_vel_A = Eigen::Vector3d(pos_param_.end_vel_vec_A[0], pos_param_.end_vel_vec_A[1], pos_param_.end_vel_vec_A[2]);
  end_acc_A = Eigen::Vector3d(pos_param_.end_acc_vec_A[0], pos_param_.end_acc_vec_A[1], pos_param_.end_acc_vec_A[2]);

  start_pos_B = Eigen::Vector3d(pos_param_.start_pos_vec_B[0], pos_param_.start_pos_vec_B[1], pos_param_.start_pos_vec_B[2]);
  start_vel_B = Eigen::Vector3d(pos_param_.start_vel_vec_B[0], pos_param_.start_vel_vec_B[1], pos_param_.start_vel_vec_B[2]);
  start_acc_B = Eigen::Vector3d(pos_param_.start_acc_vec_B[0], pos_param_.start_acc_vec_B[1], pos_param_.start_acc_vec_B[2]);
  mid_pos1_B = Eigen::Vector3d(pos_param_.mid_pos1_vec_B[0], pos_param_.mid_pos1_vec_B[1], pos_param_.mid_pos1_vec_B[2]);
  mid_vel1_B = Eigen::Vector3d(pos_param_.mid_vel1_vec_B[0], pos_param_.mid_vel1_vec_B[1], pos_param_.mid_vel1_vec_B[2]);
  mid_acc1_B = Eigen::Vector3d(pos_param_.mid_acc1_vec_B[0], pos_param_.mid_acc1_vec_B[1], pos_param_.mid_acc1_vec_B[2]);
  end_pos_B = Eigen::Vector3d(pos_param_.end_pos_vec_B[0], pos_param_.end_pos_vec_B[1], pos_param_.end_pos_vec_B[2]);
  end_vel_B = Eigen::Vector3d(pos_param_.end_vel_vec_B[0], pos_param_.end_vel_vec_B[1], pos_param_.end_vel_vec_B[2]);
  end_acc_B = Eigen::Vector3d(pos_param_.end_acc_vec_B[0], pos_param_.end_acc_vec_B[1], pos_param_.end_acc_vec_B[2]);

  start_pos_C = Eigen::Vector3d(pos_param_.start_pos_vec_C[0], pos_param_.start_pos_vec_C[1], pos_param_.start_pos_vec_C[2]);
  start_vel_C = Eigen::Vector3d(pos_param_.start_vel_vec_C[0], pos_param_.start_vel_vec_C[1], pos_param_.start_vel_vec_C[2]);
  start_acc_C = Eigen::Vector3d(pos_param_.start_acc_vec_C[0], pos_param_.start_acc_vec_C[1], pos_param_.start_acc_vec_C[2]);
  mid_pos1_C = Eigen::Vector3d(pos_param_.mid_pos1_vec_C[0], pos_param_.mid_pos1_vec_C[1], pos_param_.mid_pos1_vec_C[2]);
  mid_vel1_C = Eigen::Vector3d(pos_param_.mid_vel1_vec_C[0], pos_param_.mid_vel1_vec_C[1], pos_param_.mid_vel1_vec_C[2]);
  mid_acc1_C = Eigen::Vector3d(pos_param_.mid_acc1_vec_C[0], pos_param_.mid_acc1_vec_C[1], pos_param_.mid_acc1_vec_C[2]);
  mid_pos2_C = Eigen::Vector3d(pos_param_.mid_pos2_vec_C[0], pos_param_.mid_pos2_vec_C[1], pos_param_.mid_pos2_vec_C[2]);
  mid_vel2_C = Eigen::Vector3d(pos_param_.mid_vel2_vec_C[0], pos_param_.mid_vel2_vec_C[1], pos_param_.mid_vel2_vec_C[2]);
  mid_acc2_C = Eigen::Vector3d(pos_param_.mid_acc2_vec_C[0], pos_param_.mid_acc2_vec_C[1], pos_param_.mid_acc2_vec_C[2]);
  end_pos_C = Eigen::Vector3d(pos_param_.end_pos_vec_C[0], pos_param_.end_pos_vec_C[1], pos_param_.end_pos_vec_C[2]);
  end_vel_C = Eigen::Vector3d(pos_param_.end_vel_vec_C[0], pos_param_.end_vel_vec_C[1], pos_param_.end_vel_vec_C[2]);
  end_acc_C = Eigen::Vector3d(pos_param_.end_acc_vec_C[0], pos_param_.end_acc_vec_C[1], pos_param_.end_acc_vec_C[2]);

  start_pos_D = Eigen::Vector3d(pos_param_.start_pos_vec_D[0], pos_param_.start_pos_vec_D[1], pos_param_.start_pos_vec_D[2]);
  start_vel_D = Eigen::Vector3d(pos_param_.start_vel_vec_D[0], pos_param_.start_vel_vec_D[1], pos_param_.start_vel_vec_D[2]);
  start_acc_D = Eigen::Vector3d(pos_param_.start_acc_vec_D[0], pos_param_.start_acc_vec_D[1], pos_param_.start_acc_vec_D[2]);
  end_pos_D = Eigen::Vector3d(pos_param_.end_pos_vec_D[0], pos_param_.end_pos_vec_D[1], pos_param_.end_pos_vec_D[2]);
  end_vel_D = Eigen::Vector3d(pos_param_.end_vel_vec_D[0], pos_param_.end_vel_vec_D[1], pos_param_.end_vel_vec_D[2]);
  end_acc_D = Eigen::Vector3d(pos_param_.end_acc_vec_D[0], pos_param_.end_acc_vec_D[1], pos_param_.end_acc_vec_D[2]);

  start_pos_E = Eigen::Vector3d(pos_param_.start_pos_vec_E[0], pos_param_.start_pos_vec_E[1], pos_param_.start_pos_vec_E[2]);
  start_vel_E = Eigen::Vector3d(pos_param_.start_vel_vec_E[0], pos_param_.start_vel_vec_E[1], pos_param_.start_vel_vec_E[2]);
  start_acc_E = Eigen::Vector3d(pos_param_.start_acc_vec_E[0], pos_param_.start_acc_vec_E[1], pos_param_.start_acc_vec_E[2]);
  end_pos_E = Eigen::Vector3d(pos_param_.end_pos_vec_E[0], pos_param_.end_pos_vec_E[1], pos_param_.end_pos_vec_E[2]);
  end_vel_E = Eigen::Vector3d(pos_param_.end_vel_vec_E[0], pos_param_.end_vel_vec_E[1], pos_param_.end_vel_vec_E[2]);
  end_acc_E = Eigen::Vector3d(pos_param_.end_acc_vec_E[0], pos_param_.end_acc_vec_E[1], pos_param_.end_acc_vec_E[2]);
}
