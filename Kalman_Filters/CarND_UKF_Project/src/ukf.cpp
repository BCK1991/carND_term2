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
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
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

  n_sig_ =  2 * n_aug_ + 1;

  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  lambda_ = 3 - n_aug_;

  weights_ = VectorXd(n_sig_);
  P_ = MatrixXd::Identity(n_x_, n_x_);
  
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

		
		std::cout << "you are here PMeas1" << std::endl;
		if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
			if (fabs(x_(0)) < 0.001 && fabs(x_(1)) < 0.001){
				x_(0) = 0.001;
				x_(1) = 0.001;
			}
		}

		

		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			double rho = meas_package.raw_measurements_[0];
			double phi = meas_package.raw_measurements_[1];
			double rho_dot = meas_package.raw_measurements_[2];

			double px = rho * cos(phi);
			double py = rho * sin(phi);
			double vx = rho_dot * cos(phi);
			double vy = rho_dot * sin(phi);
			double v = sqrt( vx*vx + vy*vy ); 
			x_ << px, py, v, 0, 0;

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
	double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
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
	std::cout << "Finish" << std::endl;

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
	// Set measurement dimension
	int n_z_ = 2;
	// Create matrix for sigma points in measurement space
	// Transform sigma points into measurement space
	MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z_, n_sig_);
	std::cout << "Zsig updated lidar:" << Zsig << std::endl;
	UpdateCommon(meas_package, Zsig, n_z_);
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
	int n_z_ = 3;
	std::cout << "Radar Update 1" << std::endl;
	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z_, n_sig_);
	//transform sigma points into measurement space
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		// extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;
		if (fabs(p_x) > 0.001 || fabs(p_y) > 0.001){
			// measurement model
			Zsig(0, i) = sqrt(p_x*p_x + p_y*p_y);                        //r
			Zsig(1, i) = atan2(p_y, p_x);                                 //phi
			Zsig(2, i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
		}
		else {
			// measurement model
			Zsig(0, i) = 0.0; //sqrt(p_x*p_x + p_y*p_y);                        //r
			Zsig(1, i) = 0.0; //atan2(p_y, p_x);                                 //phi
			Zsig(2, i) = 0.0; //(p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
		}
	}
	std::cout << "Zsig updated radar:" << Zsig << std::endl;
	UpdateCommon(meas_package, Zsig, n_z_);
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
	std::cout << "Xsig_pred_ :" << Xsig_pred_ << std::endl;
	std::cout << "PredictSigmaPoints end" << std::endl;
}

void UKF::PredictMeanCovariance(){
	std::cout << "PredictMeanCovariance start" << std::endl;
	VectorXd x_pred = VectorXd(n_x_);
	x_pred.fill(0.0);

	//predict state mean
	for (int i = 0; i < n_sig_; i++){
		x_pred = x_pred + weights_(i)* Xsig_pred_.col(i);
	}

	//predict state covariance matrix
	MatrixXd P_pred = MatrixXd(n_x_, n_x_);
	P_pred.fill(0.0);
	std::cout << "x_pred :" << x_pred << std::endl;
	for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_pred;
		//angle normalization
		while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		P_pred = P_pred + weights_(i) * x_diff * x_diff.transpose();
	}
	std::cout << "P_pred :" << P_pred << std::endl;
	// Update with predictions
	x_ = x_pred;
	P_ = P_pred;
	std::cout << "PredictMeanCovariance end" << std::endl;
}

void UKF::UpdateCommon(MeasurementPackage meas_package, MatrixXd Zsig, int n_z_){
	std::cout << "UpdateCommon start" << std::endl;
	//mean predicted measurement
	z_pred_ = VectorXd(n_z_);
	z_pred_.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {
		z_pred_ = z_pred_ + weights_(i) * Zsig.col(i);
	}
	std::cout << "z_pred_ updated :" << z_pred_ << std::endl;
	//innovation covariance matrix S
	MatrixXd S = MatrixXd(n_z_, n_z_);
	S.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred_;

		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

		S = S + weights_(i) * z_diff * z_diff.transpose();
	}
	std::cout << "S updated :" << S << std::endl;
	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z_, n_z_);

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
		R = R_radar_;
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER){ // Lidar
		R = R_laser_;
	}
	S = S + R;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z_);
	Tc.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred_;
		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;

		//angle normalization
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
			while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
			while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

			//angle normalization
			while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
			while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;
		}
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}
	std::cout << "Tc updated :" << Tc << std::endl;
	std::cout << "Radar Update 2" << std::endl;
	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	VectorXd z = VectorXd(n_z_);
	z << meas_package.raw_measurements_;
	//residual
	VectorXd z_diff = z - z_pred_;
	std::cout << "z_diff updated :" << z_diff << std::endl;
	//angle normalization
	while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K*S*K.transpose();

	if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
		NIS_radar_ = z.transpose() * S.inverse() * z;
		std::cout << "NIS_radar :" << NIS_radar_ << std::endl;
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::LASER){ // Lidar
		NIS_laser_ = z.transpose() * S.inverse() * z;
		std::cout << "NIS_laser_ :" << NIS_laser_ << std::endl;
	}
	std::cout << "Common Update 3" << std::endl;
	std::cout << "x_ updated :" << x_ << std::endl;
	std::cout << "P_ updated :" << P_ << std::endl;

}
