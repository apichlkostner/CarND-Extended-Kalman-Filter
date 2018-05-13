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
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {    
    if (!is_initialized_) {
        /*****************************************************************************
         *  Initialization
         ****************************************************************************/
        initialize_ekf(measurement_pack);
    } else {    
        if (!velocity_initialized_) {
            // Initialize velocity if possible (radar measurement)
            // may be not initialized if before only lidar measurements arrived
            initialize_ekf_velocity(measurement_pack);
        }

        /*****************************************************************************
         *  Prediction
         ****************************************************************************/
        constexpr double TIME_QUANTIZATION = 1000000.0;
        double dt = (measurement_pack.timestamp_ -previous_timestamp_) / TIME_QUANTIZATION;
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
        cout << "x_ = " << ekf_.x() << endl;
        cout << "P_ = " << ekf_.P() << endl;
    }
}

void FusionEKF::initialize_ekf(const MeasurementPackage &measurement_pack) {
    previous_timestamp_ = measurement_pack.timestamp_;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        VectorXd x_state = RadarMeasurement::polar2cartesian(measurement_pack.raw_measurements_);
        // if measurement is from a radar sensor both position and velocity can be initialized
        ekf_.InitPosition(x_state);
        ekf_.InitVelocity(x_state);

        velocity_initialized_ = true;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // for lidar measurements only the position can be initialized
        ekf_.InitPosition(measurement_pack.raw_measurements_);      
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
}

void FusionEKF::initialize_ekf_velocity(const MeasurementPackage &measurement_pack) {
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        VectorXd x_state = RadarMeasurement::polar2cartesian(measurement_pack.raw_measurements_);      
        ekf_.InitPosition(x_state);
        ekf_.InitVelocity(x_state);

        velocity_initialized_ = true;
    }
}