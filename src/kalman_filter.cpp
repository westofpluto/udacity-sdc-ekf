#include "kalman_filter.h"
#include <iostream>

#define EPSILON   0.000001
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#define M_2PI            (2.0*M_PI)

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
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    if (use_joseph_form_) {
        MatrixXd IminusKH = (I - K * H_);
        P_ = IminusKH * P_ * IminusKH.transpose() + K * R_ * K.transpose();
    } else {
        P_ = (I - K * H_) * P_;
    }
}

void KalmanFilter::UpdateSingle(const int idx, const double z) {
    long x_size = x_.size();
    int numstates=x_size;
    MatrixXd Hrow=MatrixXd(1, numstates);
    for (int i=0; i<numstates;i++) {
        Hrow(0,i)=H_(idx,i);
    }

    VectorXd zvec=VectorXd(1);
    zvec << z;

    double ri = R_(idx,idx);
    VectorXd z_pred = Hrow * x_;
    VectorXd y = zvec - z_pred;
    MatrixXd Hrowt = Hrow.transpose();
    MatrixXd tmp = Hrow * P_ * Hrowt;
    double si = 1.0/(tmp(0,0) + ri);
    MatrixXd Si = MatrixXd(1,1);
    Si << si;
    MatrixXd PHt = P_ * Hrowt;
    MatrixXd K = PHt * Si;
    //new estimate
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    if (use_joseph_form_) {
        MatrixXd IminusKH = (I - K * Hrow);
        P_ = IminusKH * P_ * IminusKH.transpose() + K * ri * K.transpose();
    } else {
        P_ = (I - K * Hrow) * P_;
    }
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    float px=x_[0];
    float py=x_[1];
    float vx=x_[2];
    float vy=x_[3];
    float predrange=(float)sqrt(px*px+py*py);
    float predphi = atan2(py,px);
    //
    // handle small range
    //
    if(predrange < EPSILON) {
        predrange = EPSILON;
    }
    float preddrdt=(px*vx+py*vy)/predrange;

    VectorXd z_pred = VectorXd(3);
    z_pred << predrange, predphi, preddrdt;
 
    VectorXd y = z - z_pred;
    //
    // need to make sure y[1] is between -pi and pi
    //
    if (y[1] > M_PI) {
        y[1] -= M_2PI;
    } else if (y[1] < -M_PI) {
        y[1] += M_2PI;
    }

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    if (use_joseph_form_) {
        MatrixXd IminusKH = (I - K * H_);
        P_ = IminusKH * P_ * IminusKH.transpose() + K * R_ * K.transpose();
    } else {
        P_ = (I - K * H_) * P_;
    }
}

void KalmanFilter::UpdateEKFSingle(const int idx, const double z) {
    long x_size = x_.size();
    int numstates=x_size;
    float px=x_[0];
    float py=x_[1];
    float vx=x_[2];
    float vy=x_[3];

    VectorXd zvec=VectorXd(1);
    zvec << z;

    VectorXd z_pred=VectorXd(1);
    double z_predval=0.0;
    if (idx==0) {
        float predrange=(float)sqrt(px*px+py*py);
        //
        // handle small range
        //
        if(predrange < EPSILON) {
            predrange = EPSILON;
        }
        z_predval=predrange;
        z_pred << z_predval;
    } else if (idx==1) {
        float predphi = atan2(py,px);
        z_predval=predphi;
        z_pred << z_predval;
    } else if (idx==2) {
        float predrange=(float)sqrt(px*px+py*py);
        float preddrdt=(px*vx+py*vy)/predrange;
        z_predval=preddrdt;
        z_pred << z_predval;
    }

    VectorXd y = zvec - z_pred;
    //
    // need to make sure angle is between -pi and pi
    //
    if (idx==1) {
        if (y[0] > M_PI) {
            y[0] -= M_2PI;
        } else if (y[0] < -M_PI) {
            y[0] += M_2PI;
        }
    }

    MatrixXd Hrow=MatrixXd(1, numstates);
    for (int i=0; i<numstates;i++) {
        Hrow(0,i)=H_(idx,i);
    }
    double ri = R_(idx,idx);
    MatrixXd Hrowt = Hrow.transpose();
    MatrixXd tmp = Hrow * P_ * Hrowt;
    double si = 1.0/(tmp(0,0) + ri);
    MatrixXd Si = MatrixXd(1,1);
    Si << si;
    MatrixXd PHt = P_ * Hrowt;
    MatrixXd K = PHt * Si;
    //new estimate
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    if (use_joseph_form_) {
        MatrixXd IminusKH = (I - K * Hrow);
        P_ = IminusKH * P_ * IminusKH.transpose() + K * ri * K.transpose();
    } else {
        P_ = (I - K * Hrow) * P_;
    }
}

