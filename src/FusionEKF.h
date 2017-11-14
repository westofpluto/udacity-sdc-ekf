#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // 
  // The standard EKF uses a single iteration for the update step. By setting numIterations to a number larger than 1,
  // we are implementing a version of the Iterated Extended Kalman Filter. In this version we use a constant number of iterations
  // rather than using a convergence test
  //
  const int numIterations=1;

  //
  // Since the measurement noise covariance matrix is diagonal, we can use sequential measurement processing to
  // avoid a matrix inverse. We probably don't need to do this for our simple problem with only 2 or 3 elements
  // in the measurement vector, but it is a useful exercise
  //
  const bool sequential_measurement_updates = true;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;

  float noise_ax;
  float noise_ay;
};

#endif /* FusionEKF_H_ */
