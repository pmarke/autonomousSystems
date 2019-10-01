#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <iostream>
#include <experimental/filesystem> 
#include <fstream>
#include <functional>

namespace fs = std::experimental::filesystem;
typedef std::function<Eigen::VectorXf(const Eigen::VectorXf&  x, const Eigen::VectorXf& u, float Ts, bool addNoise)> SysFunPtr;
typedef std::function<Eigen::VectorXf(const Eigen::VectorXf&  x, const Eigen::VectorXf& u, bool addNoise)> MeasFunPtr;


class UKF {

public:

UKF(bool propogateTrue);
~UKF();


// Set private variables
void SetMeasurementCovariance(const Eigen::MatrixXf& R);
void SetProcessCovariance(const Eigen::MatrixXf& Q);
void SetErrorCovariance(const Eigen::MatrixXf& P);
void SetKalmanGain(const Eigen::MatrixXf& K);
void SetInput(const Eigen::VectorXf& u);
void SetTrueState(const Eigen::VectorXf& x);
void SetEstimateState(const Eigen::VectorXf& xh);
void SetTrueMeasurement(const Eigen::VectorXf& z);
void SetEstimateMeasurement(const Eigen::VectorXf& zh);
void SetWeightParameters(float alpha, float kappa, float beta);

// Set function pointers
void SetSystemFunctionPointer(SysFunPtr f);
void SetMeasurementFunctionPointer(MeasFunPtr g);


// Get pointer to private variable
const Eigen::MatrixXf* GetMeasurementCovariance();
const Eigen::MatrixXf* GetProcessCovariance();
const Eigen::MatrixXf* GetErrorCovariance();
const Eigen::MatrixXf* GetKalmanGain();
const Eigen::VectorXf* GetInput();
const Eigen::VectorXf* GetTrueState();
const Eigen::VectorXf* GetEstimateState();
const Eigen::VectorXf* GetTrueMeasurement();
const Eigen::VectorXf* GetEstimateMeasurement();

void Predict(float Ts);
void Update();


private:

// Generate sigma points chi_ and propagate them through the system dynamics
// defined by f_ to compute chi_b_
void GenerateSigmaPoints();

// If this flag is true, then the UKF will propogate the true dynamics
bool propogateTrue_;

// If this flag is true, sigma points will be generated
bool genSigPoints_;

Eigen::MatrixXf Q_;    // Process noise covariance
Eigen::MatrixXf R_;    // Measurement noise covariance
Eigen::MatrixXf P_;    // Error covariance
Eigen::MatrixXf K_;    // Kalman Gain

Eigen::VectorXf u_;    // Input to system
Eigen::VectorXf x_;    // True State
Eigen::VectorXf xh_;   // Estiamted State
Eigen::VectorXf z_;    // True Measurement
Eigen::VectorXf zh_;   // Estimated Measurement

// Weigth Parameters
float alpha_;
float kappa_;
float beta_;

// Pointer to the system function
// x is the state of the system
// u is the input to the system
SysFunPtr f_;

// Pointer to the measurement function
MeasFunPtr g_;

std::vector<Eigen::VectorXf> chi_;    // Sigma Points
std::vector<Eigen::VectorXf> chi_b_;  // Sigma Points passed through system dynamics. chi_b = f(chi)
std::vector<Eigen::VectorXf> chi_z_;  // Sigma Points passed through system dynamics. chi_b = f(chi)
float weight_m_;
float weight_c_;
float weight_mc_;





};
