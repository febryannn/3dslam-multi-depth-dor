#include "iostream"
#include "math.h"
#include <eigen3/Eigen/Dense>

float dt = 0.01;

using namespace Eigen;

//untuk Kalman Filter [x, y, w]T
Matrix <float, 3, 1> Xkp;

Matrix <float, 3, 3> A = (Matrix <float, 3, 3>() << 
                          1.0, 0.0, 0.0, 
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0).finished();

Matrix <float, 3, 1> Xk_prev = (Matrix <float, 3, 1>() << 
                                0.0,
                                0.0,
                                0.0).finished();

Matrix <float, 3, 3> B = (Matrix <float, 3, 3>() << 
                          dt,  0.0, 0.0, 
                          0.0, dt,  0.0,
                          0.0, 0.0, dt).finished();

Matrix <float, 3, 1> Uk = (Matrix <float, 3, 1>() << 
                                0.0,
                                0.0,
                                0.0).finished();

Matrix <float, 3, 3> Pkp;

////////////////////setting covariance prediction//////////////////
Matrix <float, 3, 3> Pk_prev = (Matrix <float, 3, 3>() << 
                                pow(0.02,2), 0.0,        0.0,
                                0.0,        pow(0.02,2), 0.0,
                                0.0,        0.0,        pow(10.0,2)).finished();

Matrix <float, 3, 3> Pk_init = (Matrix <float, 3, 3>() << 
                                pow(0.02,2), 0.0,        0.0,
                                0.0,        pow(0.02,2), 0.0,
                                0.0,        0.0,        pow(10.0,2)).finished();

Matrix <float, 3, 3> Kg;

////////////////////setting covariance measurement//////////////////
Matrix <float, 3, 3> R_init = (Matrix <float, 3, 3>() << 
                               pow(0.3,2), 0.0,        0.0,
                               0.0,        pow(0.3,2), 0.0,
                               0.0,        0.0,        pow(4.0,2)).finished();

Matrix <float, 3, 3> R = (Matrix <float, 3, 3>() << 
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0).finished();

Matrix <float, 3, 1> Yk;

Matrix <float, 3, 3> C = (Matrix <float, 3, 3>() << 
                          1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0).finished();

Matrix <float, 3, 1> yk = (Matrix <float, 3, 1>() << 
                           0.3,
                           0.08,
                           0.35).finished();

Matrix <float, 3, 1> Xk;

Matrix <float, 3, 3> Pk;

Matrix <float, 3, 3> H = (Matrix <float, 3, 3>() << 
                          1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0).finished();

MatrixXf I = MatrixXf::Identity(3,3);

Matrix <float, 3, 1> Wk = (Matrix <float, 3, 1>() << 
                           0.0,
                           0.0,
                           0.0).finished();

Matrix <float, 3, 3> Qk = (Matrix <float, 3, 3>() << 
                           pow(0.001,2), 0.0, 0.0,
                           0.0,pow(0.001,2), 0.0,
                           0.0, 0.0, pow(0.1,2)).finished();

Matrix <float, 3, 1> Zk = (Matrix <float, 3, 1>() << 
                           0.0,
                           0.0,
                           0.0).finished();

Matrix <float, 3, 3> Gps_Q = (Matrix <float, 3, 3>() << 
                              0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0).finished();