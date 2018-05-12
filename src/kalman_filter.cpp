#include "kalman_filter.h"
#include "RadarMeasurement.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
	//state covariance matrix P
	P_ = MatrixXd(4, 4);
	P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	//measurement covariance
	R_ = MatrixXd(2, 2);
	R_ << 0.0225, 0,
			  0, 0.0225;

	//measurement matrix
	H_ = MatrixXd(2, 4);
	H_ << 1, 0, 0, 0,
		  0, 1, 0, 0;

	//the initial transition matrix F_
	F_ = MatrixXd(4, 4);
	F_ << 	1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose(); // used twice
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd PHt = P_ * Ht;

  MatrixXd K = PHt * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	MatrixXd H = RadarMeasurement::CalculateJacobian(x_);
	VectorXd z_pred = RadarMeasurement::getPolar(x_);	
	VectorXd y = z - z_pred;
	// normalize phi
#if 1
	if (y(1) > M_PI)
		y(1) -= 2*M_PI;
	if (y(1) < -M_PI)
		y(1) += 2*M_PI;
#else
	y(1) = atan2(sin(y(1)), cos(y(1)));
#endif
	MatrixXd Ht = H.transpose();  // used twice
	MatrixXd S = H * P_ * Ht + R_;
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	P_ = (I - K * H) * P_;
}
