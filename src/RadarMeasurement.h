#ifndef RADARMEASUREMENT_H_
#define RADARMEASUREMENT_H_

#include "Measurement.h"

class RadarMeasurement : public Measurement {

public:
  /**
   * Constructor
   */
  RadarMeasurement() : Measurement(MatrixXd(3,3)) {
    // radar covariance matrix
    R_ << 0.09, 0, 0,
          0, 0.0009, 0,
          0, 0, 0.09;
  }

  /**
   * Destructor
   */
  virtual ~RadarMeasurement() {}

  
  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  static MatrixXd CalculateJacobian(const VectorXd& x_state);

  static VectorXd polar2cartesian(const VectorXd& meas) {
    VectorXd cartesian(4);

    float rho = meas(0);
    float phi = meas(1);
    float rho_dot = meas(2);
    
    float sin_phi = sin(phi);
    float cos_phi = cos(phi);

    cartesian << rho * cos_phi, rho * sin_phi, rho_dot * cos_phi, rho_dot * sin_phi;

    return cartesian;
  }

  static VectorXd cartesian2polar(const VectorXd& x_state) {
    VectorXd polar(3);

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float pxy2 = sqrt(px*px + py*py);

    // near the origin the polar coordinates are assumed to be 0
    if (pxy2 < 0.000001) {
      polar << 0, 0, 0;
    } else {
      polar << pxy2,
               atan2(py, px),
               (px*vx + py*vy) / pxy2;
    }

    return polar;
  }
};

#endif
