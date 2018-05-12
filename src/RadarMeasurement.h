#ifndef RADARMEASUREMENT_H_
#define RADARMEASUREMENT_H_

#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class RadarMeasurement {
public:

  // state vector
  Eigen::VectorXd x_;


  /**
   * Constructor
   */
  RadarMeasurement();

  /**
   * Destructor
   */
  virtual ~RadarMeasurement();


  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  static MatrixXd CalculateJacobian(const VectorXd& x_state);

  static VectorXd getCartesian(const VectorXd& meas) {
    VectorXd cartesian(4);

    float rho = meas(0);
    float phi = meas(1);
    float rho_dot = meas(2);
    
    float sin_phi = sin(phi);
    float cos_phi = cos(phi);

    cartesian << rho * cos_phi, rho * sin_phi, rho_dot * cos_phi, rho_dot * sin_phi;

    return cartesian;
  }

  static VectorXd getPolar(const VectorXd& x_state) {
    VectorXd polar(3);

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float pxy2 = sqrt(px*px + py*py);

    if (pxy2 < 0.000001) {
      std::cout << "pxy2 too small: " << pxy2 << std::endl << flush;

      polar << 0, 0, 0;
    } else {
      //cout << "py = " << py << " px = " << px << " atan2 = " << atan2(py, px) << endl << flush;
      polar << pxy2, atan2(py, px), (px*vx + py*vy) / pxy2;
    }

    return polar;
  }

};

#endif
