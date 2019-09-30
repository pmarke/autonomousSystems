#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>
#include <random>
#include <iostream>
#include <experimental/filesystem> 
#include <fstream>
#include <UKF.h>


namespace fs = std::experimental::filesystem;
const float kPI = 3.14159;

class Sim
{

public:

Sim(int num_landmarks);
~Sim();

void Simulate();

private:


void InitLog();         // Init all log information
void InitLandmarks();   // Init all landmarks
void InitSystem();      // Init all system parameters
Eigen::VectorXf SystemFunction(const Eigen::VectorXf&  x, const Eigen::VectorXf& u, float Ts, bool addNoise);
Eigen::VectorXf MeasFunction(const Eigen::VectorXf&  x, const Eigen::VectorXf& u, bool addNoise);
void GetInputAndProcessCovariance(Eigen::Vector2f& u, Eigen::Matrix2f& Q, float t);

// Log Data
void LogData();
void LogMeasurement();


// Filter
UKF ukf_;


// Covariance coefficients
float alpha1_ = 0.1;
float alpha2_ = 0.01;
float alpha3_ = 0.01;
float alpha4_ = 0.1;

float sigma_r_= 0.1;
float sigma_phi_ = 0.05;

// Time variables
float t_=0;          // Current time
float Ts_=0.1;       // Time step
float tf_=20;        // Final time

std::vector<Eigen::Vector2f> landmarks_;  // The position of the landmarks
int num_landmarks_;                       // Number of landmarks
unsigned int id_;                         // Current id of landmark

const Eigen::MatrixXf* R_;
const Eigen::MatrixXf* Q_;
const Eigen::MatrixXf* P_;
const Eigen::MatrixXf* K_;
const Eigen::VectorXf* u_;
const Eigen::VectorXf* x_;
const Eigen::VectorXf* xh_;
const Eigen::VectorXf* z_;
const Eigen::VectorXf* zh_;


// Log Data
std::ofstream log_K_;   // Kalman Gain          
std::ofstream log_t_;   // Time             
std::ofstream log_x_;   // True state
std::ofstream log_xh_;  // Estimate state
std::ofstream log_u_;   // Input
std::ofstream log_P_;   // Error covariance
std::ofstream log_m_;   // Measurement log 


// Used to add noise to measurements
std::default_random_engine gen_;
std::normal_distribution<double> randn_{0,1};

};