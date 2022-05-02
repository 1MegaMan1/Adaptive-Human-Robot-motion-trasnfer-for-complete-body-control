
/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
  https://github.com/hmartiro/kalman-cpp/blob/master/kalman.cpp
*/

#include <iostream>
#include <stdexcept>
#include <eigen3/Eigen/Dense>
#include <kinect_v2/kalmanfilter.hpp>
KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
//initialization lists
  : A(A), C(C), Q(Q), R(R), P0(P),
    m(C.rows()), n(A.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) 
{
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() 
{
  x_hat.setOnes();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;

  A << 1, dt, dt, 
       0, 1, dt, 
       0, 0, 1;

  C << 1, 0, 0;

  Q << .05, .05, .0, 
       .05, .05, .0, 
       .00, .00, .0;

  R <<  5;

  P << .1, .1   , .1,
       .1, 10000, 10,
       .1, 10   , 100;

}

void KalmanFilter::update(double pos) 
{
  Eigen::VectorXd y(1);
  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  if(!(std::isnan(pos))) y(0) = pos;
  else pos = x_hat(0);
  x_hat_new = A * x_hat;//Predicted state estimate
  P = A*P*A.transpose() + Q;//Predicted Error Covariance
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();//kalman gain
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(double y, double dt, const Eigen::MatrixXd A) 
{
  this->A = A;
  this->dt = dt;
  update(y);
}
