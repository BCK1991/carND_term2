#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  
	is_initialized_ = false;
	// if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  n_x_ = x_.size();

  n_aug_ =  n_x_ + 2;

  n_sig_ = 1 + 2 * n_aug_;

  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(n_sig_);

  
  R_radar_ = MatrixXd(3, 3);
  R_radar_ <<	std_radr_*std_radr_, 0, 0,
				0, std_radphi_*std_radphi_, 0,
				0, 0, std_radrd_*std_radrd_;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ <<	std_laspx_*std_laspx_, 0,
				0, std_laspy_*std_laspy_;

  Xsig_aug_ = MatrixXd(n_aug_, n_sig_);

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
	std::cout << "you are here PMeas" << std::endl;
	if (!is_initialized_) {

		P_ << 0.5, 0, 0, 0, 0,
			0, 0.5, 0, 0, 0,
			0, 0, 0.5, 0, 0,
			0, 0, 0, 0.5, 0,
			0, 0, 0, 0, 0.5;
		std::cout << "you are here PMeas1" << std::endl;
		if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;

		}

		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			double rho = meas_package.raw_measurements_[0];
			double phi = meas_package.raw_measurements_[1];
			double rho_dot = meas_package.raw_measurements_[2];

			double px = rho * cos(phi);
			double py = rho * sin(phi);
			
			x_ << px, py, rho_dot, 0, 0;

		}
		std::cout << "you are here PMeas2" << std::endl;
		//Init weights
		weights_(0) = lambda_ / (lambda_ + n_aug_);
		for (int i = 1; i < weights_.size(); i++) {
			weights_(i) = 0.5 / (n_aug_ + lambda_);
		}

		std::cout << "you are here PMeas2.5" << std::endl;
		time_us_ = meas_package.timestamp_;
		std::cout << "you are here PMeas2.6" << std::endl;
		is_initialized_ = true;
		cout << "Init" << endl;
		cout << "x_" << x_ << endl;
		return;
	}
	std::cout << "you are here PMeas3" << std::endl;
	double dt = (meas_package.timestamp_ - time_us_) ;
	time_us_ = meas_package.timestamp_;
	std::cout << "you are here PMEnd" << std::endl;

	Prediction(dt);
	std::cout << "UpdateEnter" << std::endl;
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
		//cout << "Radar " << measurement_pack.raw_measurements_[0] << " " << measurement_pack.raw_measurements_[1] << endl;
		UpdateRadar(meas_package);
	}
	if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
		//cout << "Lidar " << measurement_pack.raw_measurements_[0] << " " << measurement_pack.raw_measurements_[1] << endl;
		UpdateLidar(meas_package);
	}


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	std::cout << "you are here prediction" << std::endl;
	//create augmanted state matrix
	x_aug_ = VectorXd(n_aug_);
	// mean of the process noises are zero 
	x_aug_ << x_, 0, 0;

	//create augmanted covariance matrix
	P_aug_ = MatrixXd(n_aug_, n_aug_);

	P_aug_.fill(0.0);
	P_aug_.topLeftCorner(n_x_, n_x_) = P_;
	P_aug_(5, 5) = std_a_*std_a_;
	P_aug_(6, 6) = std_yawdd_*std_yawdd_;
	std::cout << "you are here" << std::endl;
	std::cout << P_aug_ << std::endl;
	GenerateSigmaPoints();
	PredictSigmaPoints(delta_t);
	PredictMeanCovariance();
	

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(3, n_sig_);
	//transform sigma points into measurement space
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		// extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                        //r
		Zsig(1, i) = atan2(p_y, p_x);                                 //phi
		Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
	}

	//mean predicted measurement
	z_pred_ = VectorXd(3);
	z_pred_.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {
		z_pred_ = z_pred_ + weights_(i) * Zsig.col(i);
	}

	//innovation covariance matrix S
	MatrixXd S = MatrixXd(3, 3);
	S.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred_;

		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(3, 3);
	R << std_radr_*std_radr_, 0, 0,
		0, std_radphi_*std_radphi_, 0,
		0, 0, std_radrd_*std_radrd_;
	S = S + R;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x, 3);

	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred_;
		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		Tc = Tc + weights(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//residual
	VectorXd z_diff = z - z_pred_;

	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S*K.transpose();
}

void UKF::GenerateSigmaPoints() {
	//x_k,k1 = xk,k
	//x_k,k2,3 = xk,k + sqrt((lambda + n_x) * P_k,k)
	//x_k,k2,3 = xk,k - sqrt((lambda + n_x) * P_k,k)

	//Calculate square root of P
	MatrixXd A = P_aug_.llt().matrixL();
	std::cout << "GenSigPts 1" << std::endl;
	//std::cout << Xsig_aug_.size() << std::endl;
	//std::cout << x_aug_.size() << std::endl;
	//Assign x_ as first column
	Xsig_aug_.col(0) = x_aug_;
	std::cout << Xsig_aug_ << std::endl;
	//set remaining sigma points
	for (int i = 0; i < n_x_; i++){
		Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * A.col(i);
		//std::cout << i << std::endl;
		Xsig_aug_.col(i + 1 + n_x_) = x_aug_ - sqrt(lambda_ + n_aug_) * A.col(i);
	}
	std::cout << "GenSigPts end" << std::endl;
}

void UKF::PredictSigmaPoints(double delta_t) {

	std::cout << "PredictSigmaPoints start" << std::endl;
	for (int i = 0; i< n_sig_; i++)
	{
		//extract values for better readability
		double p_x = Xsig_aug_(0, i);
		double p_y = Xsig_aug_(1, i);
		double v = Xsig_aug_(2, i);
		double yaw = Xsig_aug_(3, i);
		double yawd = Xsig_aug_(4, i);
		double nu_a = Xsig_aug_(5, i);
		double nu_yawdd = Xsig_aug_(6, i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
		}
		else {
			px_p = p_x + v*delta_t*cos(yaw);
			py_p = p_y + v*delta_t*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd*delta_t;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
		py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
		v_p = v_p + nu_a*delta_t;

		yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
		yawd_p = yawd_p + nu_yawdd*delta_t;

		//write predicted sigma point into right column
		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = yaw_p;
		Xsig_pred_(4, i) = yawd_p;
	}
	std::cout << "PredictSigmaPoints end" << std::endl;
}

void UKF::PredictMeanCovariance(){

	x_.fill(0.0);
	//predict state mean
	for (int i = 0; i < n_sig_; i++){
		x_ = x_ + weights_(i)* Xsig_pred_.col(i);
	}

	//predict state covariance matrix
	P_.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
	}

}
