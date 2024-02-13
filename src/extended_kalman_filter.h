#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>

//This is an Extended Kalman Filter for use on the KITTI dataset for 2D odometry
class ExtendedKalmanFilter {
public:
    // Constructor
    ExtendedKalmanFilter();

    // Destructor
    virtual ~ExtendedKalmanFilter();

    //Update the state based on observation of (x, y)
    void Update(const Eigen::VectorXd &observation);

    //Get the current state
    Eigen::VectorXd GetState() const;

    //Get the current covariance
    Eigen::MatrixXd GetCovariance() const;

    //Get the current state transition matrix
    Eigen::MatrixXd GetStateTransition() const;

    //Propagte based on the state transition matrix
    void Propagate();

    //Set the state transition matrix
    void SetStateTransition(const Eigen::MatrixXd &state_transition);

    //Set the process noise covariance
    void SetProcessNoise(const Eigen::MatrixXd &process_noise);

    //Set the observation model
    void SetObservationModel(const Eigen::MatrixXd &observation_model);

    //Set the observation noise covariance
    void SetObservationNoise(const Eigen::MatrixXd &observation_noise);

    //Set the initial state
    void SetState(const Eigen::VectorXd &state);

    //Set the initial covariance
    void SetCovariance(const Eigen::MatrixXd &covariance);

private:
    //State
    Eigen::VectorXd state_;

    //Covariance
    Eigen::MatrixXd covariance_;

    //State transition matrix
    Eigen::MatrixXd state_transition_;

    //Process noise covariance
    Eigen::MatrixXd process_noise_;

    //Observation model
    Eigen::MatrixXd observation_model_;

    //Observation noise covariance
    Eigen::MatrixXd observation_noise_;
};

#endif // EXTENDED_KALMAN_FILTER_H