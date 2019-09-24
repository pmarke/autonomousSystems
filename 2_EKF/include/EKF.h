#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>
#include <random>
#include <iostream>
#include <experimental/filesystem> 
#include <fstream>

namespace fs = std::experimental::filesystem;

const float kPI = 3.14159;

class EKF
{

public: 

EKF(int num_landmarks);
~EKF();

void sim();

private:


// The prediction step for the EKF.
// u= [v, w] and is the input
// Ts is the time step
void Predict(const Eigen::Vector2f& u, float Ts);

// Update the EKF when a measurement z comes. 
// z = [r,th], the range and angle to the measurement
// id, the unique id of the landmark
void Update(const int id);

// Step the true states foward
void Step(const Eigen::Vector2f& u, float t, float Ts);

// Log Data
void LogData();

void LogMeasurement(const Eigen::Vector2f& z);

// Initialize landmarks
void InitLandmarks();

Eigen::Vector2f GetMeasurement(int id);

Eigen::Vector2f GetInput(float t);

Eigen::Vector3f x_;          // True states
Eigen::Vector3f xh_{0,0,0};  // Estimated states
Eigen::Matrix3f P_;          // Estimate Covariance
Eigen::Matrix2f Q_;          // Measurement Covariance
Eigen::Matrix2f S_;          // Innovation term
Eigen::Matrix<float,3,2> K_; // Kalman Gain


Eigen::Vector2f u_;          // Input into the system
// Covariance coefficients
float alpha1_ = 0.1;
float alpha2_ = 0.01;
float alpha3_ = 0.01;
float alpha4_ = 0.1;

float sigma_r_= 0.1;
float sigma_phi_ = 0.05*1;

// Time variables
float t_;         // Current time
float Ts_;        // Time step
float tf_;        // Final time

std::vector<Eigen::Vector2f> landmarks_;  // The position of the landmarks

// Used to add noise to measurements
std::default_random_engine gen_;
std::normal_distribution<double> randn_{0,1};

int num_landmarks_;

// Log Data
std::ofstream log_K_;                 
std::ofstream log_t_;                
std::ofstream log_x_;  
std::ofstream log_xh_;  
std::ofstream log_u_;  
std::ofstream log_P_; 
std::ofstream log_m_;   // Measurement log 

};