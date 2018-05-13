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
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
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

    previous_timestamp_ = measurement_pack.timestamp_;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // if measurement is from a radar sensor both position and velocity can be initialized
      ekf_.x_ = RadarMeasurement::polar2cartesian(measurement_pack.raw_measurements_);
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
      VectorXd pos = RadarMeasurement::polar2cartesian(measurement_pack.raw_measurements_);
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

  // prediction only if time has changed till last update
  constexpr double MINIMUM_PREDICT_TIME = 0.001;

  if (dt > MINIMUM_PREDICT_TIME) {
    ekf_.Predict(dt);
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateRadar(measurement_pack.raw_measurements_);
  } else {
    ekf_.UpdateLidar(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
