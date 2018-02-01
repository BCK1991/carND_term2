#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

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
  /**
  TODO:
    * predict the state
  */
	std::cout << "Checkpoint 0-0" << std::endl;
	x_ = F_ * x_;
	//update state transition coveriance based on state transition matrix and process covariance matrix (based on delta t) 
	VectorXd F_t = F_.transpose();
	P_ = F_ * P_ * F_t + Q_;
	std::cout << "Checkpoint 0" << std::endl;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	//calculate error between prediction and the ground truth
	std::cout << "Checkpoint 1-0" << std::endl;
	int z_size = z.size();
	MatrixXd I = MatrixXd::Identity(z_size, z_size);
	VectorXd y = z - H_ * x_; 
	VectorXd H_transpose = H_.transpose();
	VectorXd S = H_ * P_ * H_transpose + R_;
	VectorXd S_inv = S.inverse();
	VectorXd K = P_ * H_transpose * S_inv;
	x_ = x_ + K * y;
	P_ = (I - K * H_) * P_;
	std::cout << "Checkpoint 1" << std::endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	int z_size = z.size();
	MatrixXd I = MatrixXd::Identity(z_size, z_size);

	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	float rho = sqrt(px*px + py*py);
	float theta = atan2(py, px);
	float rho_dot = (px * vx + py * vy) / ( sqrt(px * px + py * py) );

	VectorXd h = VectorXd(3);
	h << rho, theta, rho_dot;

	VectorXd y = VectorXd(3);
	y = z - h;

	//check limits (-pi,pi)
	while (y(1) > M_PI || y(1) < -M_PI) {
		if (y(1) > M_PI) {
			y(1) -= M_PI;
		}
		else {
			y(1) += M_PI;
		}
	}

	y = z - H_ * x_;
	VectorXd H_transpose = H_.transpose();
	VectorXd S = H_ * P_ * H_transpose + R_;
	VectorXd S_inv = S.inverse();
	VectorXd K = P_ * H_transpose * S_inv;

	x_ = x_ + K * y;
	P_ = (I - K * H_) * P_;
	std::cout << "Checkpoint 2" << std::endl;
}
