#include "kalman_filter.h"
#include <math.h>

#include <iostream>
using namespace std;

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
  x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
	VectorXd y 			= z - z_pred;
	MatrixXd Ht 		= H_.transpose();
	MatrixXd S 			= H_ * P_ * Ht + R_;
	MatrixXd Si 		= S.inverse();
	MatrixXd PHt 		= P_ * Ht;
	MatrixXd K 			= PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double c1 = sqrt(x_(0)*x_(0)+x_(1)*x_(1));
  VectorXd hx = VectorXd(3) ;
  hx << 1,1,1;

  if (c1 > 0.000001 && fabs(x_(0)) >0.000001) {
    hx(0) = c1;
    hx(1) = atan2( x_(1), x_(0) );
  //  cout << hx(1) << '\n';
    hx(2) = (x_(2)*x_(0) + x_(3)*x_(1) )/c1;
  }

	VectorXd y 			= z - hx;

    // Angle normalization. Very important!!!
    while (y(1)>PI) {
      y(1) -= 2 * PI;
    }
    while (y(1)<-PI) {
      y(1) += 2 * PI;
    }

	MatrixXd Ht 		= H_.transpose();
	MatrixXd S 			= H_ * P_ * Ht + R_;
	MatrixXd Si 		= S.inverse();
	MatrixXd PHt 		= P_ * Ht;
	MatrixXd K 			= PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}
