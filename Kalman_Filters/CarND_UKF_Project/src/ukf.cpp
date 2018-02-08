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

  n_aug_ = 2 * n_x_ + 1;

  Xsig_pred_ = MatrixXd(n_x_, n_aug_);

  lambda_ = 3 - n_x_;

  weights_ = VectorXd(n_aug_);

  
  R_radar_ = MatrixXd(3, 3);
  R_radar_ <<	std_radr_*std_radr_, 0, 0,
				0, std_radphi_*std_radphi_, 0,
				0, 0, std_radrd_*std_radrd_;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ <<	std_laspx_*std_laspx_, 0,
				0, std_laspy_*std_laspy_;

  Xsig_aug_ = MatrixXd(n_x_, n_aug_);

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

		P_ = MatrixXd::Identity(5, 5);
		P_ = 0.5 * P_;
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
		long timeStamp = meas_package.timestamp_;
		
		is_initialized_ = true;
		//cout << "Init" << endl;
		//cout << "x_" << x_ << endl;
		return;
	}
	std::cout << "you are here PMeas3" << std::endl;
	time_us_ = (meas_package.timestamp_ - timeStamp) / 1000000.0;
	timeStamp = meas_package.timestamp_;
	std::cout << "you are here PMEnd" << std::endl;
	Prediction(time_us_);

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
	P_aug_(5, 5) = std_a_;
	P_aug_(6, 6) = std_yawdd_;
	std::cout << "you are here" << std::endl;
	std::cout << P_aug_ << std::endl;
	GenerateSigmaPoints();

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
	
	

}

void UKF::GenerateSigmaPoints() {
	//x_k,k1 = xk,k
	//x_k,k2,3 = xk,k + sqrt((lambda + n_x) * P_k,k)
	//x_k,k2,3 = xk,k - sqrt((lambda + n_x) * P_k,k)

	//Calculate square root of P
	MatrixXd A = P_aug_.llt().matrixL();

	//Assign x_ as first column
	Xsig_aug_.col(0) = x_aug_;

	//set remaining sigma points
	for (int i = 0; i < n_x_; i++){
		Xsig_aug_.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
		Xsig_aug_.col(i + 1 + n_x_) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
	}

}
