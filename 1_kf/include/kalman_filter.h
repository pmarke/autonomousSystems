#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <iostream>
#include <fstream> 
#include <time.h>
#include <experimental/filesystem> 


namespace fs = std::experimental::filesystem;

namespace kf {

// Implements a Kalman Filter for a LTV system of type
// x_dot(t) = Ax(t) + Bu(t) +Gw(t) or x_n = A_n*x_n-1 + B_n*u_n +Gn*wn
// y = C(t)x(t)
// or
//

class KalmanFilter
{

public:

  KalmanFilter(bool continuousSystem);
  ~KalmanFilter();

  // Set the initial estimate of xh
  void SetInitialEstimateState(const Eigen::VectorXf& x);

  // Set the Observation Matrix
  void SetObservationMatrix(const Eigen::MatrixXf& C);
  
  // Set the system matrix A(t) for continuous systems
  void SetSystemMatrixCont(const Eigen::MatrixXf& Ac);

  // Set the system matrix A_n  for discrete systems
  void SetSystemMatrixDiscrete(const Eigen::MatrixXf& Ad);

  // Set the input matrix B(t) for continuous system
  void SetInputMatrixCont(const Eigen::MatrixXf& Bc);

  // Set the input matrix B_n for discrete time
  void SetInputMatrixDiscrete(const Eigen::MatrixXf& Bd);

  // Set the input matrix G(t) for continuous system
  void SetNoiseMatrixCont(const Eigen::MatrixXf& Gc);

  // Set the input matrix Gd for discrete time
  void SetNoiseMatrixDiscrete(const Eigen::MatrixXf& Gd);


  // Set the initial covariance matrix
  void SetCovarianceMatrix(const Eigen::MatrixXf& P);

  // Propagates the estimated state, xh, forward.
  // Ts is the time passed since last prediction
  // u is the camera's acceleration in the inertial frame
  void Predict(float Ts, const Eigen::VectorXf& u);

  // Update the estimated state when a measurement is received
  void Update(const Eigen::VectorXf& y);

  void SetR(const Eigen::MatrixXf& R);
  void SetQCont(const Eigen::MatrixXf& Qc);
  void SetQDiscrete(const Eigen::MatrixXf& Qd);

  // Return a pointer to the estimated state
  const Eigen::VectorXf* GetEstimatedState();

  // Return a pointer to the discrete input matrix
  const Eigen::MatrixXf* GetInputMatrixDiscrete();

  // Return a pointer to the discrete input matrix
  const Eigen::MatrixXf* GetSystemMatrixDiscrete();

  // Return a pointer to the discrete input matrix
  const Eigen::MatrixXf* GetNoiseMatrixDiscrete();

  // Return a pointer to the discrete process noise
  const Eigen::MatrixXf* GetProcessNoiseCovDiscrete();

  // Transforms the system from continuous to discrete
  void Discretize(float Ts);

private:





  float Ts_ = 0;                    // Time step between measurements



  Eigen::VectorXf x_;      // Estimated State Vector
  Eigen::MatrixXf Bc_;     // Input Matrix continuous
  Eigen::MatrixXf Bd_;     // Input Matrix Discrete
  Eigen::MatrixXf C_;      // Observation Matrix
  Eigen::MatrixXf Ac_;     // System Matrix
  Eigen::MatrixXf Ad_;     // System Matrix
  Eigen::MatrixXf P_;      // Covariance Error Matrix
  Eigen::MatrixXf R_;      // Measurement Covariance
  Eigen::MatrixXf Qc_;      // System Covariance continuous
  Eigen::MatrixXf Qd_;      // System Covariance discrete
  Eigen::MatrixXf Gc_;      // Process Noise Matrix Continuous
  Eigen::MatrixXf Gd_;      // Process Noise Matrix

  bool cont_sys_;                 // flag to indicate that user is using a continous system
  bool need_discretize_;          // flag to indicate that the system needs to be discretized



};

}