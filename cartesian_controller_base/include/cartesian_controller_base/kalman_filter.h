#ifndef KALMAN_FILTER_H_INCLUDED
#define KALMAN_FILTER_H_INCLUDED


#include <Eigen/Dense>
#include <memory>
namespace cartesian_controller_base{
class KalmanFilter
{
public:

    KalmanFilter(const Eigen::Vector3d& initial_state, const Eigen::Matrix3d& initial_covariance,
              const Eigen::Matrix3d& transition_matrix, const Eigen::Vector3d& observation_matrix, 
              const Eigen::Matrix3d& process_noise, const double& measurement_noise);

    Eigen::Vector3d predict();
    Eigen::Vector3d update(const Eigen::Vector3d& measurement);

private:
    Eigen::Vector3d state_;
    Eigen::Matrix3d covariance_;
    Eigen::Matrix3d transition_matrix_;
    Eigen::Vector3d observation_matrix_;
    Eigen::Matrix3d process_noise_;
    double measurement_noise_;
};

}

#endif // KALMAN_FILTER_H_INCLUDED