#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>
#include <random>
#include <iostream>
#include <experimental/filesystem> 
#include <fstream>
#include <particle_filter.h>


namespace fs = std::experimental::filesystem;
//const float kPI = 3.14159;

class Sim
{

public:

Sim(int num_landmarks, int M);
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
//void LogMeasurement();


// Filter
ParticleFilter pf_;


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
int M_ = 1000;


const Eigen::VectorXf* u_;
const Eigen::VectorXf* x_;
const Eigen::Vector3f* xh_;
const Eigen::VectorXf* z_;
const Eigen::VectorXf* chi_;
const Eigen::VectorXf* zh_;


// Log Data
std::ofstream log_chi_;   // Kalman Gain
std::ofstream log_t_;   // Time             
std::ofstream log_x_;   // True state
std::ofstream log_xh_;  // Estimate state
std::ofstream log_u_;   // Input
std::ofstream log_m_;   // Measurement log 


// Used to add noise to measurements
std::default_random_engine gen_;
std::normal_distribution<double> randn_{0,1};

};