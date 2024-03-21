#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorWaypoint {
    private:
		double _qp_cost;
		Eigen::MatrixXd _Q;
		Eigen::VectorXd _Px, _Py, _Pz;
    public:
        TrajectoryGeneratorWaypoint();
        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd CoordGeneration(
            const Eigen::MatrixXd &Path,
            const Eigen::VectorXd &Time,
            const double &limit_d,
            const double &limit_vel,
            const double &limit_acc,
            const double &limit_jerk
            );

        Eigen::VectorXd OsqpEigenSolve(const Eigen::MatrixXd &Path, const Eigen::VectorXd &Time, 
                                       const double &limit_d, const double &limit_vel, const double &limit_acc,
                                       const double &limit_jerk);
};
        

#endif
