#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "System.h"
#include "RadarMeasurement.h"
#include "LidarMeasurement.h"

class KalmanFilter {
public:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  System system_;

  RadarMeasurement radar_measurement_;

  LidarMeasurement lidar_measurement_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(double dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateLidar(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateRadar(const Eigen::VectorXd &z);

  void InitPosition(const Eigen::VectorXd x);

  void InitVelocity(const Eigen::VectorXd v);

  const Eigen::VectorXd x() const { return x_; };
  const Eigen::MatrixXd P() { return P_; };
  const Eigen::MatrixXd Q() { return Q_; };
};

#endif /* KALMAN_FILTER_H_ */
