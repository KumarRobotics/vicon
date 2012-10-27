#include "vicon_odom/filter.h"
#include <Eigen/LU> // For matrix inverse

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::KalmanFilter(const KalmanFilter::State_t &state,
                           const KalmanFilter::ProcessCov_t &initial_cov,
                           const KalmanFilter::ProcessCov_t &process_noise,
                           const KalmanFilter::MeasurementCov_t &meas_noise)
{
  x = state;
  P = initial_cov;
  Q = process_noise;
  R = meas_noise;
}

void KalmanFilter::initialize(const State_t &state,
                              const ProcessCov_t &initial_cov,
                              const ProcessCov_t &process_noise,
                              const MeasurementCov_t &meas_noise)
{
  x = state;
  P = initial_cov;
  Q = process_noise;
  R = meas_noise;
}

void KalmanFilter::processUpdate(double dt)
{
  ProcessCov_t A = ProcessCov_t::Identity();
  A.topRightCorner<3,3>() = Eigen::Vector3d(dt, dt, dt).asDiagonal();

  x = A*x;
  P = A*P*A.transpose() + Q;
}

void KalmanFilter::measurementUpdate(const Measurement_t &meas, double dt)
{
  Eigen::Matrix<double, n_meas, n_states> H;
  H.setZero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  H(2, 2) = 1;

  const Eigen::Matrix<double, n_states, n_meas> K = P * H.transpose() *
      (H*P*H.transpose() + R).inverse();
  const Measurement_t inno = meas - H*x;
  x += K*inno;
  P = (ProcessCov_t::Identity() - K*H) * P;
}

void KalmanFilter::setProcessNoise(const ProcessCov_t &process_noise)
{
  Q = process_noise;
}

void KalmanFilter::setMeasurementNoise(const MeasurementCov_t &meas_noise)
{
  R = meas_noise;
}

const KalmanFilter::State_t &KalmanFilter::getState(void)
{
  return x;
}

const KalmanFilter::ProcessCov_t &KalmanFilter::getProcessNoise(void)
{
  return P;
}
