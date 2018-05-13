#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  void initialize_ekf(const MeasurementPackage &measurement_pack);
  void initialize_ekf_velocity(const MeasurementPackage &measurement_pack);

  KalmanFilter& ekf() { return ekf_; }

private:
  /**
    * Kalman Filter update and prediction math lives in here.
    */
  KalmanFilter ekf_;
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_ = false;
  bool velocity_initialized_ = false;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
};

#endif /* FusionEKF_H_ */
