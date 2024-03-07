#include <iostream>
#include <string.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <traj_utils/polynomial_traj.h>
#include <traj_utils/planning_visualization.h>
#include <ius_msgs/Trajectory.h>
#include <vector>
#include <std_msgs/Int8.h>
#include <bspline_opt/bspline_optimizer.h>
#include <ius_msgs/Trajectory.h>


using namespace std;

// Trajectory point define 

/*Task A   -- Mission for pillar -- traj init  */

Eigen::Vector3d start_pos_A,start_vel_A,start_acc_A;
Eigen::Vector3d mid_pos1_A,mid_vel1_A,mid_acc1_A;
Eigen::Vector3d mid_pos2_A,mid_vel2_A,mid_acc2_A;
Eigen::Vector3d mid_pos3_A,mid_vel3_A,mid_acc3_A;
Eigen::Vector3d mid_pos4_A,mid_vel4_A,mid_acc4_A;
Eigen::Vector3d end_pos_A,end_vel_A,end_acc_A;

auto trajA_1 = PolynomialTraj::one_segment_traj_gen(start_pos_A, start_vel_A, start_acc_A,
                                                  mid_pos1_A, mid_vel1_A, mid_acc1_A, 3.0);
auto trajA_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_A, mid_vel1_A, mid_acc1_A,
                                                  mid_pos2_A, mid_vel2_A, mid_acc2_A, 3.0);
auto trajA_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_A, mid_vel2_A, mid_acc2_A,
                                                  mid_pos3_A, mid_vel3_A, mid_acc3_A, 3.0);
auto trajA_4 = PolynomialTraj::one_segment_traj_gen(mid_pos3_A, mid_vel3_A, mid_acc3_A,
                                                  mid_pos4_A, mid_vel4_A, mid_acc4_A, 3.0); 
auto trajA_5 = PolynomialTraj::one_segment_traj_gen(mid_pos4_A, mid_vel4_A, mid_acc4_A,
                                                  end_pos_A, end_vel_A, end_acc_A, 3.0);

 /*Task B   -- Mission for box -- traj init  */

Eigen::Vector3d start_pos_B,start_vel_B,start_acc_B;
Eigen::Vector3d mid_pos1_B,mid_vel1_B,mid_acc1_B;
Eigen::Vector3d end_pos_B,end_vel_B,end_acc_B;

auto trajB_1 = PolynomialTraj::one_segment_traj_gen(start_pos_B, start_vel_B, start_acc_B,
                                                  mid_pos1_B, mid_vel1_B, mid_acc1_B, 3.0);
auto trajB_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_B, mid_vel1_B, mid_acc1_B,
                                                    end_pos_B, end_vel_B, end_acc_B, 3.0);
auto trajB_3 = PolynomialTraj::one_segment_traj_gen(mid_pos1_B, mid_vel1_B, mid_acc1_B,
                                                    end_pos_B, end_vel_B, end_acc_B, 3.0);

/*Task C   -- Mission for maze -- traj init  */

Eigen::Vector3d start_pos_C,start_vel_C,start_acc_C;
Eigen::Vector3d mid_pos1_C,mid_vel1_C,mid_acc1_C;
Eigen::Vector3d mid_pos2_C,mid_vel2_C,mid_acc2_C;
Eigen::Vector3d end_pos_C,end_vel_C,end_acc_C;

auto trajC_0 = PolynomialTraj::one_segment_traj_gen(end_pos_B, end_vel_B, end_acc_B,
                                                  start_pos_C, start_vel_C, start_acc_C, 3.0);

auto trajC_1 = PolynomialTraj::one_segment_traj_gen(start_pos_C, start_vel_C, start_acc_C,
                                                  mid_pos1_C, mid_vel1_C, mid_acc1_C, 3.0);

auto trajC_2 = PolynomialTraj::one_segment_traj_gen(mid_pos1_C, mid_vel1_C, mid_acc1_C,
                                                    mid_pos2_C, mid_vel2_C, mid_acc2_C, 3.0);

auto trajC_3 = PolynomialTraj::one_segment_traj_gen(mid_pos2_C, mid_vel2_C, mid_acc2_C,
                                                    end_pos_C, end_vel_C, end_acc_C, 3.0);
                   
 /*Task D   -- Mission for loop -- traj init  */

Eigen::Vector3d start_pos_D,start_vel_D,start_acc_D;
Eigen::Vector3d end_pos_D,end_vel_D,end_acc_D;

auto trajD_1 = PolynomialTraj::one_segment_traj_gen(end_pos_C, end_vel_C, end_acc_C,
                                                  start_pos_D, start_vel_D, start_acc_D, 3.0);
auto trajD_2 = PolynomialTraj::one_segment_traj_gen(start_pos_D, start_vel_D, start_acc_D,
                                                    end_pos_D, end_vel_D, end_acc_D, 3.0);

/*Task E   -- Mission for m_loop -- traj init  */

Eigen::Vector3d start_pos_E,start_vel_E,start_acc_E;
Eigen::Vector3d end_pos_E,end_vel_E,end_acc_E;

auto trajE_1 = PolynomialTraj::one_segment_traj_gen(end_pos_D, end_vel_D, end_acc_D,
                                                  start_pos_E, start_vel_E, start_acc_E, 3.0);
auto trajE_2 = PolynomialTraj::one_segment_traj_gen(start_pos_E, start_vel_E, start_acc_E,
                                                    end_pos_E, end_vel_E, end_acc_E, 3.0);

