#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

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
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0;

  noise_ax = 9.0;
  noise_ay = 9.0;

  ekf_.F_ = MatrixXd(4, 4);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


   cout << "ProcessMeasurement called " << endl;

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
    cout << "First measurement EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float range = measurement_pack.raw_measurements_[0];
      float phi=measurement_pack.raw_measurements_[1];
      float px=range*cos(phi);
      float py=range*sin(phi);
      //
      // least norm solution: vx = (px/r)*drdt, vy=(py/r)*drdt
      // 
      float drdt=measurement_pack.raw_measurements_[2];
      float pxor=px/range;
      float pyor=py/range;
      float vx=pxor*drdt; 
      float vy=pyor*drdt; 

      ekf_.x_ << px, py, vx, vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    //
    // we also need to create the initial state covariance matrix.
    // This represents the initial state uncertainty and usually requires some physical intuition regarding
    // the problem at hand. In our case, the measurement error covariance components are pretty small
    // so we should have a pretty good idea of the initial positions. We can use 1.0 for the initial error
    // uncertainty there. We really don't know what the velocity is, but we can be conservative and use 50*50=2500
    // for the velocity error uncertainties.
    //
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 2500, 0,
             0, 0, 0, 2500;

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    cout << "End of first measurement EKF: " << endl;
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
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  cout << "Computing update " << endl;

  MatrixXd G = MatrixXd(4,2);
  float dt2over2=dt*dt/2.0;
  //float dt3over2=dt*dt2over2;
  //float dt4over3=dt*dt*dt*dt/3.0;
  G << dt2over2, 0., 0., dt2over2, dt, 0., 0., dt;

  cout << "Finished with G " << endl;

  MatrixXd Qnu =  MatrixXd(2,2);
  Qnu << noise_ax, 0., 0., noise_ay;
  cout << "Finished with Qnu " << endl;
  ekf_.Q_ = G*Qnu*G.transpose();
  cout << "Finished with ekf_.Q_ " << endl;

  ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;

  cout << "Before Predict " << endl;
  ekf_.Predict();
  cout << "After Predict " << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.R_ = R_radar_;
    //
    // generalize so that we can use more than one iteration (IEKF)
    //
    for (int i=0; i<numIterations; i++) {
        if (sequential_measurement_updates) {
            for (int idx=0;idx<3;idx++) {
                Hj_ = tools.CalculateJacobian(ekf_.x_);
                ekf_.H_ = Hj_;
                ekf_.UpdateEKFSingle((const int)idx,measurement_pack.raw_measurements_[idx]);
            }
        } else {
            Hj_ = tools.CalculateJacobian(ekf_.x_);
            ekf_.H_ = Hj_;
            ekf_.UpdateEKF(measurement_pack.raw_measurements_);
        }
    }
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    if (sequential_measurement_updates) {
        for (int idx=0;idx<2;idx++) {
            ekf_.UpdateSingle((const int)idx,measurement_pack.raw_measurements_[idx]);
        }
    } else {
        ekf_.Update(measurement_pack.raw_measurements_);

    }
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
