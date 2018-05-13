#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
	//state covariance matrix P
	P_ = MatrixXd(4, 4);
	P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	// state vector
	x_ = VectorXd::Zero(4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict(double dt) {
	MatrixXd F = system_.F(dt);
	MatrixXd Q = system_.Q(dt);
	x_ = F * x_;
	P_ = F * P_ * F.transpose() + Q;
}

void KalmanFilter::UpdateLidar(const VectorXd &z) {
	const MatrixXd H = lidar_measurement_.H();
	VectorXd z_pred = H * x_;
	VectorXd y = z - z_pred;

	MatrixXd Ht = H.transpose(); // used twice
	MatrixXd S = H * P_ * Ht + lidar_measurement_.R();
	MatrixXd PHt = P_ * Ht;

	MatrixXd K = PHt * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	P_ = (I - K * H) * P_;
}

void KalmanFilter::UpdateRadar(const VectorXd &z) {
	const MatrixXd R = radar_measurement_.R();
	const MatrixXd H = RadarMeasurement::CalculateJacobian(x_);
	const VectorXd z_pred = RadarMeasurement::cartesian2polar(x_);

	VectorXd y = z - z_pred;
	// normalize phi
	y(1) = Tools::NormalizeAnge(y(1));

	MatrixXd Ht = H.transpose();  // used twice
	MatrixXd S = H * P_ * Ht + R;
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	P_ = (I - K * H) * P_;
}

void KalmanFilter::InitPosition(const Eigen::VectorXd x) {
	constexpr int xpos = 0;
	x_(0) = x(xpos);
	x_(1) = x(xpos + 1);
}

void KalmanFilter::InitVelocity(const Eigen::VectorXd x) {
	constexpr int vpos = 2;
	x_(0) = x(vpos);
	x_(1) = x(vpos + 1);
}