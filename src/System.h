#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class System {
    public:
    System() {
        // system matrix
        F_ = MatrixXd(4, 4);
        
        constexpr double dt_init = 0.001;

        F_ << 1, 0, dt_init, 0,
              0, 1, 0,       dt_init,
              0, 0, 1,       0,
              0, 0, 0,       1;
    }

    const MatrixXd& F(double dt) {
        F_(0, 2) = dt;
        F_(1, 3) = dt;

        return F_;
    }

    static MatrixXd Q(double dt) {
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

    protected:
    MatrixXd F_;
};

#endif