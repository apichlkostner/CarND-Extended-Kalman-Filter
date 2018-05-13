#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Measurement {
protected:
    //measurement covariance matrix
    Eigen::MatrixXd R_;
    // measurement matrix
    Eigen::MatrixXd H_;

public:
    Measurement() : R_(2,2), H_(2,4) {
        R_ << 0.0225, 0,
              0, 0.0225;

        //measurement matrix
        H_ = MatrixXd(2, 4);
        H_ << 1, 0, 0, 0,
              0, 1, 0, 0;
    }

    Measurement(MatrixXd R) : R_(R) {};
    
    virtual ~Measurement(){}

    // returns the covariance matrix
    const MatrixXd& R() {
        return R_;
    }

    const MatrixXd& H() {
        return H_;
    }
};

#endif