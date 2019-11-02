#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>
#include <random>
#include <iostream>
#include <experimental/filesystem> 
#include <fstream>
#include <limits>
#include <cmath>

namespace fs = std::experimental::filesystem;

const float kPI = 3.14159;

class EKF_SLAM
{

public: 

EKF_SLAM(int num_landmarks, float vel, float radius);
~EKF_SLAM();

void Sim();

private:

void InitEnvironment();

// Returns the location of the x an y position of the landmark in 
// vector x specified by landmark_id
Eigen::Vector2f getLandmark(const Eigen::VectorXf& x, int landmark_id);

// Sets the location of the x an y position of the landmark in 
// vector x specified by landmark_id
void setLandmark(Eigen::VectorXf& x, int landmark_id, const Eigen::Vector2f& m);

// The system function. 
// x: the state vector
// u: the input
// Ts: the time step
Eigen::VectorXf getg(const Eigen::VectorXf& x, const Eigen::VectorXf& u, float Ts);

// Computes the jacobian of the system funciton g
// x: the state vector
// u: the input
// Ts: the time step
Eigen::MatrixXf getG(const Eigen::VectorXf& x, const Eigen::VectorXf& u, float Ts);

// Compute the process noise to system map
// x: the state vector
// u: the input
// Ts: the time step
Eigen::MatrixXf getV(const Eigen::VectorXf& x, const Eigen::VectorXf& u, float Ts);

// Compute the process noise covariance
// u: the input
Eigen::MatrixXf getM(const Eigen::VectorXf& u);

// Propogate the true state vector in time
// u: the input
// Ts: the time step
void PropogateTrue(const Eigen::VectorXf& u, float Ts);

// The predict phase of the EKF
// u: the input
// Ts: the time step
void Predict(const Eigen::VectorXf& u, float Ts);

// Compute the measurement
// landmark_id: the landmarks id
Eigen::Vector2f getMeasurement(const Eigen::VectorXf& x, int landmark_id, bool flag_true, bool flag_noise);

// Compute the small H
// landmark_id: the landmarks id
Eigen::Matrix<float,2,5> getSmallH(int landmark_id);

// Checks to see if you are seeing a landmark for the first time
// z: the measurement to the landmark
// landmark_id: the landmarks id
void NewLandmark(const Eigen::VectorXf& z, int landmark_id);

// The update phase of the EKF
// z: the measurement to the landmark
// landmark_id: the landmarks id
void Update(const Eigen::VectorXf& z, int landmark_id);

void LogTrueState();
void LogEstState();
void LogErrorCovariance();
void LogMeasurement(const Eigen::Vector2f& z);
void LogData();

// Environment Parameters
float vel_;
float radius_;
float angular_vel_;
int num_landmarks_;



Eigen::VectorXf x_;          // True states
Eigen::VectorXf xh_;         // Estimated states
Eigen::MatrixXf P_;          // Estimate Covariance
Eigen::Matrix2f Q_;          // Measurement Covariance
Eigen::MatrixXf S_;          // Innovation term
Eigen::MatrixXf K_;          // Kalman Gain


Eigen::Vector2f u_;          // Input into the system
// Covariance coefficients
float alpha1_ = 0.1;
float alpha2_ = 0.01;
float alpha3_ = 0.01;
float alpha4_ = 0.1;

float sigma_r_= 0.1;
float sigma_phi_ = 0.05;

float fov_ = 2*kPI;          // Field of view for the sensor

// Time variables
float t_;         // Current time
float Ts_;        // Time step
float tf_;        // Final time

std::vector<Eigen::Vector2f> landmarks_;  // The position of the landmarks
std::vector<bool> landmark_seen_;         // Indicates if it has been seen before

// Used to add noise to measurements
std::default_random_engine gen_;
std::normal_distribution<double> randn_{0,1};



// Log Data
// std::ofstream log_K_;                 
std::ofstream log_t_;                
std::ofstream log_x_;  
std::ofstream log_xh_;  
std::ofstream log_u_;  
std::ofstream log_P_; 
std::ofstream log_m_;   // Measurement log 
std::ofstream log_landmarks_;;

};