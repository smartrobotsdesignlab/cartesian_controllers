#include <cartesian_controller_base/kalman_filter.h>

namespace cartesian_controller_base{

KalmanFilter::KalmanFilter(const Eigen::Vector3d& initial_state, const Eigen::Matrix3d& initial_covariance,
                        const Eigen::Matrix3d& transition_matrix, const Eigen::Vector3d& observation_matrix, 
                        const Eigen::Matrix3d& process_noise, const double& measurement_noise)
{
    state_ = initial_state;
    covariance_ = initial_covariance;
    transition_matrix_ = transition_matrix;
    observation_matrix_ = observation_matrix;
    process_noise_ = process_noise;
    measurement_noise_ = measurement_noise;
}

Eigen::Vector3d KalmanFilter::predict()
{
    state_ = transition_matrix_ * state_;
    covariance_ = transition_matrix_ * covariance_ * transition_matrix_.transpose() + process_noise_;
    return state_;
}

Eigen::Vector3d KalmanFilter::update(const Eigen::Vector3d& measurement)
{
    double S = observation_matrix_.transpose() * covariance_ * observation_matrix_ + measurement_noise_;
    Eigen::Vector3d kalman_gain = (covariance_ * observation_matrix_) / S;
    state_ = state_ + kalman_gain * (measurement(0) - observation_matrix_.transpose() * state_);
    covariance_ = (Eigen::Matrix3d::Identity(state_.size(), state_.size()) - kalman_gain * observation_matrix_.transpose()) * covariance_;
    return state_;
}

} // namespace