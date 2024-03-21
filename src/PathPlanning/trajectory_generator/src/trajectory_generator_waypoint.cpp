#include "trajectory_generator/trajectory_generator_waypoint.h"
#include "OsqpEigen/OsqpEigen.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::CoordGeneration(
            const Eigen::MatrixXd &Path,
            const Eigen::VectorXd &Time,
            const double &limit_d,
            const double &limit_vel,
            const double &limit_acc,
            const double &limit_jerk
            )
{
    Eigen::VectorXd coord_x = OsqpEigenSolve(Path.col(0), Time, limit_d, limit_vel, limit_acc, limit_jerk);
    Eigen::VectorXd coord_y = OsqpEigenSolve(Path.col(1), Time, limit_d, limit_vel, limit_acc, limit_jerk);
    Eigen::VectorXd coord_z = OsqpEigenSolve(Path.col(2), Time, limit_d, limit_vel, limit_acc, limit_jerk);
    // Path points
    Eigen::MatrixXd coord(coord_x.size(), 3);
    coord << coord_x, coord_y, coord_z;
    
    return coord;
}

Eigen::VectorXd TrajectoryGeneratorWaypoint::OsqpEigenSolve(const Eigen::MatrixXd &Path, const Eigen::VectorXd &Time, 
                                                            const double &limit_d, const double &limit_vel, 
                                                            const double &limit_acc, const double &limit_jerk) {
    int m = Time.size();
    int steps = Time.maxCoeff() * 10;
    // ROS_INFO("steps: %d", steps);
    int len = 3 * m * (steps+1);
    Eigen::MatrixXd Aul = Eigen::MatrixXd::Identity(len-3, len);
    // first segment
    double dt = Time(0) / double(steps);
    for (int i=0; i<steps; ++i) {
        Aul(3*i, 3*i+1) = dt;
        Aul(3*i+1, 3*i+2) = dt;
        Aul(3*i, 3*i+3) = -1.0;
        Aul(3*i+1, 3*i+4) = -1.0;
        Aul(3*i+2, 3*i+5) = -1.0;
    }
    // other segments
    for (int seg=1; seg<m; ++seg) {
        dt = Time(seg) / double(steps);
        int idx = 3 * (steps+1) * seg - 3;
        Aul(idx, idx+3) = -1.0;
        Aul(idx+1, idx+4) = -1.0;
        Aul(idx+2, idx+5) = -1.0;
        for (int i=0; i<steps; ++i) {
            Aul(idx+3+3*i, idx+3+3*i+1) = dt;
            Aul(idx+3+3*i+1, idx+3+3*i+2) = dt;
            Aul(idx+3+3*i, idx+3+3*i+3) = -1.0;
            Aul(idx+3+3*i+1, idx+3+3*i+4) = -1.0;
            Aul(idx+3+3*i+2, idx+3+3*i+5) = -1.0;
        }
    }

    // the up left part of matrix Ac
    Eigen::MatrixXd Aur = Eigen::MatrixXd::Zero(len-3, m*steps);
    // first segment
    dt = Time(0) / double(steps);
    for (int i=0; i<steps; ++i) {
        Aur(3*i+2, i) = dt;
    }
    // other segments
    for (int seg=1; seg<m; ++seg) {
        dt = Time(seg) / double(steps);
        // calculate the first index of every segment
        int idx = 3*steps + 3*(steps+1)*(seg-1);
        int idy = seg * steps;
        for (int i=0; i<steps; ++i) {
            Aur(idx+3*i+5, idy+i) = dt;
        }
    }

    // state and control input
    len = 4*m*steps + 3*m;
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Identity(len, len);
    Eigen::MatrixXd Ac(Aul.rows()+len, len);
    Ac << Aul, Aur, Ad;
    Eigen::SparseMatrix<double> Acs = Ac.sparseView();

    // compute l and u
    Eigen::VectorXd lu0 = Eigen::VectorXd::Zero(3*(m*steps+m-1));
    Eigen::VectorXd ldx_state = Eigen::VectorXd::Ones(3*m*(steps+1)) * -limit_acc;
    // every segment
    for (int i=0; i<m; ++i) {
        // every step
        int idx = 3 * i * (steps+1);
        ldx_state(idx) = Path(i, 0);
        ldx_state(idx+1) = -0.0;
        ldx_state(idx+2) = -0.0;
        for (int j=1; j<steps; ++j) {
            ldx_state(idx+3*j) = Path(i, 0) + double(j)/double(steps)*(Path(i+1, 0)-Path(i, 0)) - limit_d;
            ldx_state(idx+3*j+1) = -limit_vel;
        }
        ldx_state(idx+3*steps) = Path(i+1, 0);
        ldx_state(idx+3*steps+1) = -0.0;
        ldx_state(idx+3*steps+2) = -0.0;
    }
    Eigen::VectorXd ldx_control = Eigen::VectorXd::Ones(m*steps) * -limit_jerk;
    Eigen::VectorXd lux_state = Eigen::VectorXd::Ones(3*m*(steps+1)) * limit_acc;
    // every segment
    for (int i=0; i<m; ++i) {
        // every step
        int idx = 3 * i * (steps+1);
        lux_state(idx) = Path(i, 0);
        lux_state(idx+1) = 0.0;
        lux_state(idx+2) = 0.0;
        for (int j=1; j<steps; ++j) {
            lux_state(idx+3*j) = Path(i, 0) + double(j)/double(steps)*(Path(i+1, 0)-Path(i, 0)) + limit_d;
            lux_state(idx+3*j+1) = limit_vel;
        }
        lux_state(idx+3*steps) = Path(i+1, 0);
        lux_state(idx+3*steps+1) = 0.0;
        lux_state(idx+3*steps+2) = 0.0;
    }
    Eigen::VectorXd lux_control = Eigen::VectorXd::Ones(m*steps) * limit_jerk;
    len = lu0.rows()+ldx_state.rows()+ldx_control.rows();
    Eigen::VectorXd lx(len), ux(len);
    lx << lu0, ldx_state, ldx_control;
    ux << lu0, lux_state, lux_control;
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(4*m*steps+3*m, 4*m*steps+3*m);
    P.block(3*m*(steps+1), 3*m*(steps+1), m*steps, m*steps) = Eigen::MatrixXd::Identity(m*steps, m*steps);
    Eigen::SparseMatrix<double> Ps = P.sparseView();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(4*m*steps+3*m);

    // OsqpEigen solver
    Eigen::MatrixXd None = Eigen::MatrixXd::Zero(0, 0);
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(4*m*steps+3*m);
    solver.data()->setNumberOfConstraints(7*m*steps+6*m-3);
    if (!solver.data()->setHessianMatrix(Ps)) std::cout << "Failed to set hessian matrix." << std::endl;
    if (!solver.data()->setGradient(q)) std::cout << "Failed to set gradient." << std::endl;
    if (!solver.data()->setLinearConstraintsMatrix(Acs)) std::cout << "Failed to set linear constraints matrix." << std::endl;
    if (!solver.data()->setLowerBound(lx)) std::cout << "Failed to set lower bound." << std::endl;
    if (!solver.data()->setUpperBound(ux)) std::cout << "Failed to set upper bound." << std::endl;

    if (!solver.initSolver()) {
        std::cout << "Failed to initialize solver" << std::endl;
    }
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cout << "Failed to solve the problem." << std::endl;
    }

    Eigen::VectorXd solution = solver.getSolution();
    len = m * (steps+1);
    Eigen::VectorXd coord(len);
    for (int i=0; i<len; ++i) {
        coord[i] = solution(3*i);
    }

    return coord;
}
