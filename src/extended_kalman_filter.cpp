#include <iostream>
#include "extended_kalman_filter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    //Initialize state
    state_ = Eigen::VectorXd(4);
    state_ << 0, 0, 0, 0;

    //Initialize covariance
    covariance_ = Eigen::MatrixXd(4, 4);
    covariance_ << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;

    //Initialize state transition matrix
    state_transition_ = Eigen::MatrixXd(4, 4);
    state_transition_ << 1, 0, 1, 0,
                         0, 1, 0, 1,
                         0, 0, 1, 0,
                         0, 0, 0, 1;

    //Initialize process noise covariance
    process_noise_ = Eigen::MatrixXd(4, 4);
    process_noise_ << 0.1, 0, 0, 0,
                      0, 0.1, 0, 0,
                      0, 0, 0.1, 0,
                      0, 0, 0, 0.1;

    //Initialize observation model
    observation_model_ = Eigen::MatrixXd(2, 4);
    observation_model_ << 1, 0, 0, 0,
                          0, 1, 0, 0;

    //Initialize observation noise covariance
    observation_noise_ = Eigen::MatrixXd(2, 2);
    observation_noise_ << 0.1, 0,
                          0, 0.1;
}

