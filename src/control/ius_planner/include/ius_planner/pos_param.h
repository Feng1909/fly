#pragma once

#include <ros/ros.h>

class Posparam
{
public:
    Posparam() {}

    void load_param(ros::NodeHandle& nh) {
        nh.getParam("start_pos_A", start_pos_vec_A);
        nh.getParam("start_vel_A", start_vel_vec_A);
        nh.getParam("start_acc_A", start_acc_vec_A);

        nh.getParam("mid_pos1_A", mid_pos1_vec_A);
        nh.getParam("mid_vel1_A", mid_vel1_vec_A);
        nh.getParam("mid_acc1_A", mid_acc1_vec_A);

        nh.getParam("mid_pos2_A", mid_pos2_vec_A);
        nh.getParam("mid_vel2_A", mid_vel2_vec_A);
        nh.getParam("mid_acc2_A", mid_acc2_vec_A);

        nh.getParam("mid_pos3_A", mid_pos3_vec_A);
        nh.getParam("mid_vel3_A", mid_vel3_vec_A);
        nh.getParam("mid_acc3_A", mid_acc3_vec_A);

        nh.getParam("mid_pos4_A", mid_pos4_vec_A);
        nh.getParam("mid_vel4_A", mid_vel4_vec_A);
        nh.getParam("mid_acc4_A", mid_acc4_vec_A);

        nh.getParam("end_pos_A", end_pos_vec_A);
        nh.getParam("end_vel_A", end_vel_vec_A);
        nh.getParam("end_acc_A", end_acc_vec_A);

        nh.getParam("start_pos_B", start_pos_vec_B);
        nh.getParam("start_vel_B", start_vel_vec_B);
        nh.getParam("start_acc_B", start_acc_vec_B);

        nh.getParam("mid_pos1_B", mid_pos1_vec_B);
        nh.getParam("mid_vel1_B", mid_vel1_vec_B);
        nh.getParam("mid_acc1_B", mid_acc1_vec_B);

        nh.getParam("end_pos_B", end_pos_vec_B);
        nh.getParam("end_vel_B", end_vel_vec_B);
        nh.getParam("end_acc_B", end_acc_vec_B);

        nh.getParam("start_pos_C", start_pos_vec_C);
        nh.getParam("start_vel_C", start_vel_vec_C);
        nh.getParam("start_acc_C", start_acc_vec_C);

        nh.getParam("mid_pos1_C", mid_pos1_vec_C);
        nh.getParam("mid_vel1_C", mid_vel1_vec_C);
        nh.getParam("mid_acc1_C", mid_acc1_vec_C);

        nh.getParam("mid_pos2_C", mid_pos2_vec_C);
        nh.getParam("mid_vel2_C", mid_vel2_vec_C);
        nh.getParam("mid_acc2_C", mid_acc2_vec_C);
        
        nh.getParam("end_pos_C", end_pos_vec_C);
        nh.getParam("end_vel_C", end_vel_vec_C);
        nh.getParam("end_acc_C", end_acc_vec_C);

        nh.getParam("start_pos_D", start_pos_vec_D);
        nh.getParam("start_vel_D", start_vel_vec_D);
        nh.getParam("start_acc_D", start_acc_vec_D);

        nh.getParam("end_pos_D", end_pos_vec_D);
        nh.getParam("end_vel_D", end_vel_vec_D);
        nh.getParam("end_acc_D", end_acc_vec_D);

        nh.getParam("start_pos_E", start_pos_vec_E);
        nh.getParam("start_vel_E", start_vel_vec_E);
        nh.getParam("start_acc_E", start_acc_vec_E);

        nh.getParam("end_pos_E", end_pos_vec_E);
        nh.getParam("end_vel_E", end_vel_vec_E);
        nh.getParam("end_acc_E", end_acc_vec_E);

    }

    // mission A pos_param
    std::vector<double> start_pos_vec_A,start_vel_vec_A,start_acc_vec_A;
    std::vector<double> mid_pos1_vec_A,mid_vel1_vec_A,mid_acc1_vec_A;
    std::vector<double> mid_pos2_vec_A,mid_vel2_vec_A,mid_acc2_vec_A;
    std::vector<double> mid_pos3_vec_A,mid_vel3_vec_A,mid_acc3_vec_A;
    std::vector<double> mid_pos4_vec_A,mid_vel4_vec_A,mid_acc4_vec_A;
    std::vector<double> end_pos_vec_A,end_vel_vec_A,end_acc_vec_A;

    // mission B pos_param
    std::vector<double> start_pos_vec_B,start_vel_vec_B,start_acc_vec_B;
    std::vector<double> mid_pos1_vec_B,mid_vel1_vec_B,mid_acc1_vec_B;
    std::vector<double> end_pos_vec_B,end_vel_vec_B,end_acc_vec_B;

    // mission C pos_param
    std::vector<double> start_pos_vec_C,start_vel_vec_C,start_acc_vec_C;
    std::vector<double> mid_pos1_vec_C,mid_vel1_vec_C,mid_acc1_vec_C;
    std::vector<double> mid_pos2_vec_C,mid_vel2_vec_C,mid_acc2_vec_C;
    std::vector<double> end_pos_vec_C,end_vel_vec_C,end_acc_vec_C;

    // mission D pos_param
    std::vector<double> start_pos_vec_D,start_vel_vec_D,start_acc_vec_D;
    std::vector<double> end_pos_vec_D,end_vel_vec_D,end_acc_vec_D;

    // mission E pos_param
    std::vector<double> start_pos_vec_E,start_vel_vec_E,start_acc_vec_E;
    std::vector<double> end_pos_vec_E,end_vel_vec_E,end_acc_vec_E;

};
