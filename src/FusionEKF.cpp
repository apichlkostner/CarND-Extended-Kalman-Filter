#include "FusionEKF.h"
#include "RadarMeasurement.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd::Zero(4);

    previous_timestamp_ = measurement_pack.timestamp_;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // if measurement is from a radar sensor both position and velocity can be initialized
      ekf_.x_ = RadarMeasurement::getCartesian(measurement_pack.raw_measurements_);
      velocity_initialized_ = true;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // for lidar measurements only the position can be initialized
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // Initialize velocity with first measurement
  if (!velocity_initialized_) {
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      VectorXd pos = RadarMeasurement::getCartesian(measurement_pack.raw_measurements_);
      ekf_.x_(2) = pos(2);
      ekf_.x_(3) = pos(3);
    }

    velocity_initialized_ = true;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double dt = (measurement_pack.timestamp_ -previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

#if 0
  auto Q = [dt]() -> MatrixXd {
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    //set the acceleration noise components
    constexpr float noise_ax = 9;
    constexpr float noise_ay = 9;
    
    MatrixXd Q(4,4);
    
    Q <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
          0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
          dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
          0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    return Q;
  };

	auto F = [dt](const MatrixXd& F) -> MatrixXd {
    MatrixXd Fn(F);
    Fn(0, 2) = dt;
    Fn(1, 3) = dt;

    return Fn;
  };
#endif

  // prediction only if time has changed till last update
  if (dt > 0.00001)
    ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_);
  } else {
    ekf_.Update(measurement_pack.raw_measurements_, R_laser_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
