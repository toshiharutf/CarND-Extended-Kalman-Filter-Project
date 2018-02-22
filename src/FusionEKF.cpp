#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

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

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

// laser model transformation matrix
  H_laser_ << 1, 0, 0, 0,
  			      0, 1, 0, 0;

// Hj is the Jacobian of a nonlinear model for the radar sensor.
// Hj is computed online in every iteration by calling a function in tools.cpp

// Measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      // Radar measurements data structure : rho, theta, rhodot
   float rho = measurement_pack.raw_measurements_[0]; // range
   float phi = measurement_pack.raw_measurements_[1]; // bearing
   float rho_dot = measurement_pack.raw_measurements_[2]; // rho rate's of change

   // polar to cartesian transformation.
   // TODO: is there a function in the Eigen library that can deal with the rotation?

   float x = rho * cos(phi);
   float y = rho * sin(phi);
   float vx = rho_dot * cos(phi);
   float vy = rho_dot * sin(phi);
   ekf_.x_ << x, y, vx , vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Remember that LIDAR only provides the position of an object, not its velocity
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // Initial covariance matrix
      ekf_.P_ = MatrixXd(4, 4);
      ekf_.P_ << 1, 0, 0, 0,
  			   0, 1, 0, 0,
  			   0, 0, 1000, 0,
          0, 0, 0, 1000;

      previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    std::cout << "Kalman filter initialization successful" << '\n';
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

   //compute the time elapsed between the current and previous measurements
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
   previous_timestamp_ = measurement_pack.timestamp_;

   //std::cout << "F computation begin!" << '\n';

   ekf_.F_ = MatrixXd(4, 4);
   ekf_.F_ << 1,0,dt,0,
     			  0,1,0,dt,
     			  0,0,1,0,
     			  0,0,0,1;

  //std::cout << "F computation OK!" << '\n';

  // Noise values from the task
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  // precomputed values to optimize the computation time
  float dt_2 = dt * dt; //dt^2
  float dt_3 = dt_2 * dt; //dt^3
  float dt_4 = dt_3 * dt; //dt^4

 	ekf_.Q_ = MatrixXd(4, 4);
  //std::cout << "Q computation begin!" << '\n';
 	ekf_.Q_ << dt_4/4*noise_ax , 0                     ,dt_3/2*noise_ax , 0                   ,
 			  0                    , dt_4/4*noise_ay,                     0 , dt_3/2*noise_ay,
 			  dt_3/2*noise_ax , 0                      , dt_2*noise_ax ,                    0,
 			  0                    , dt_3/2*noise_ay,                     0 , dt_2*noise_ay  ;

  //std::cout << "Q computation end!" << '\n';

  ekf_.Predict();
  //std::cout << "Prediction" << '\n';

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // The radar transfer function is nonlinear, so the Jacobian is used to linearize it
    // around the mesured point.

  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  ekf_.R_ = R_radar_;
  ekf_.UpdateEKF(measurement_pack.raw_measurements_);


  //std::cout << "Radar update" << '\n';

} else {
    // Laser updates
//  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){

    ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  //std::cout << "LIDAR update" << '\n';
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
