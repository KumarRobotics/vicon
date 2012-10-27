#include <Eigen/Core>

class KalmanFilter
{
 public:
  static const int n_states = 6;
  static const int n_meas = 3;
  typedef Eigen::Matrix<double, n_states, 1> State_t;
  typedef Eigen::Matrix<double, n_states, n_states> ProcessCov_t;
  typedef Eigen::Matrix<double, n_meas, 1> Measurement_t;
  typedef Eigen::Matrix<double, n_meas, n_meas> MeasurementCov_t;

  KalmanFilter();
  KalmanFilter(const State_t &state,
               const ProcessCov_t &initial_cov,
               const ProcessCov_t &process_noise,
               const MeasurementCov_t &meas_noise);

  void initialize(const State_t &state,
                  const ProcessCov_t &initial_cov,
                  const ProcessCov_t &process_noise,
                  const MeasurementCov_t &meas_noise);

  void processUpdate(double dt);
  void measurementUpdate(const Measurement_t &meas, double dt);

  void setProcessNoise(const ProcessCov_t &process_noise);
  void setMeasurementNoise(const MeasurementCov_t &meas_noise);

  const State_t &getState(void);
  const ProcessCov_t &getProcessNoise(void);

 private:
  State_t x;
  ProcessCov_t P, Q;
  MeasurementCov_t R;
};
