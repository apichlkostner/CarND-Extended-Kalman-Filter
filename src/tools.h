#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  static double NormalizeAnge(double angle) {
#if 1
    if (angle > M_PI) {
      angle -= 2*M_PI;
    }
    else if (angle < -M_PI) {
      angle += 2*M_PI;
    }
#else
    angle = atan2(angle, angle));
#endif
    return angle;
  }
};

#endif /* TOOLS_H_ */
