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


class State {

public:

// State(const Eigen::Vector3f& x, int num_landmarks, int num_particles, unsigned int tmp) {


//   landmarks_seen_ = std::vector<bool>(num_landmarks,false);
//   weight_ = 1.0f/num_particles;
//   num_landmarks_ = num_landmarks;

//   stacked_states_ = Eigen::VectorXf::Ones(3+2*num_landmarks_)*tmp;
//   stacked_covariance_ = Eigen::VectorXf::Ones(4*num_landmarks_)*tmp;
//   // stacked_states_.head(3) = x;

//     // std::cerr << "mem C: " << stacked_covariance_.data() << std::endl;



//   // new (&x_) Eigen::Map<Eigen::Vector3f>(stacked_states_.data(),3);


  
//   for (unsigned int k = 0; k < num_landmarks_; k++) {

//     landmarks_.emplace_back(stacked_states_.data()+3+2*k,2);


//     P_.emplace_back(stacked_covariance_.data()+4*k,2,2);
//     P_[k] << 100,0, 0, 100;

//     // std::cerr << "mem: " << P_[k].data() << std::endl;


//   }
// }

State(const State& other) {
  landmarks_seen_ = other.landmarks_seen_;
  weight_ = other.weight_;
  num_landmarks_ = other.num_landmarks_;
  stacked_states_ = other.stacked_states_;
  stacked_covariance_ = other.stacked_covariance_;

  for (unsigned int k = 0; k < num_landmarks_; k++) {

    // landmarks_.emplace_back(stacked_states_.data()+3+2*k,2);
    landmarks_.emplace_back(stacked_states_.data()+3+2*k);


    // P_.emplace_back(stacked_covariance_.data()+4*k,2,2);
    P_.emplace_back(stacked_covariance_.data()+4*k);
    // P_[k] << 100,0, 0, 100;

    // std::cerr << "data: " << P_[k] << std::endl;


  }

}

// ~State() {
//   // delete stacked_states_;
//   stacked_states_.resize(0);
//   // delete stacked_covariance_;
//   stacked_covariance_.resize(0);
//   // std::cerr << "here de: " << std::endl;


//   for (unsigned int k = 0; k < num_landmarks_; k++) {

//     new (&landmarks_[k]) Eigen::Map<Eigen::Vector2f>(NULL,2);


//     new (&P_[k]) Eigen::Map<Eigen::Matrix2f>(NULL,2,2);
//     // P_[k] << 100,0, 0, 100;

//     // std::cerr << "data: " << P_[k] << std::endl;


//   }
// }

State& operator=(State other) {
  std::cerr << "here " << std::endl;
}

State(const Eigen::Vector3f& x, int num_landmarks, int num_particles) {


  landmarks_seen_ = std::vector<bool>(num_landmarks,false);
  weight_ = 1.0f/num_particles;
  num_landmarks_ = num_landmarks;

  stacked_states_ = Eigen::VectorXf::Zero(3+2*num_landmarks_);
  stacked_covariance_ = Eigen::VectorXf::Zero(4*num_landmarks_);
  stacked_states_.head(3) = x;


  // new (&x_) Eigen::Map<Eigen::Vector3f>(stacked_states_.data(),3);


  
  for (unsigned int k = 0; k < num_landmarks_; k++) {

    // landmarks_.emplace_back(stacked_states_.data()+3+2*k,2);
    landmarks_.emplace_back(stacked_states_.data()+3+2*k);


    // P_.emplace_back(stacked_covariance_.data()+4*k,2,2);
    P_.emplace_back(stacked_covariance_.data()+4*k);
    P_[k] << 100,0, 0, 100;


  }
}

void AddWeight(float weight) {
  if (!weight_flag_) {
    weight_ = weight;
    weight_flag_ = true;
  }
  else
    weight_*=weight;
}


Eigen::VectorXf stacked_states_;           // The robot's pose stacked with landmark's pos
Eigen::VectorXf stacked_covariance_;       // The stacked covariance of all of the landmarks

// Eigen::Map<Eigen::Vector3f> x_{NULL};                  // The robot's pose, x = [px, py, theta]
std::vector<Eigen::Map<Eigen::Vector2f>> landmarks_;   // The landmarks position
std::vector<bool> landmarks_seen_;                     // Indicates if a specific landmark has been seen
std::vector<Eigen::Map<Eigen::Matrix2f>> P_;           // The landmarks error covariance
float weight_;                                         // The current weight of the particle
int num_landmarks_;
bool weight_flag_ = false;


};

class FastSlam {


public:

FastSlam(int num_landmarks, float fov, int num_particles, float vel, float radius);
~FastSlam();
void Sim();

private:

void InitLog();
void CloseLogs();
void InitEnvrionment();
void InitParticles();

void CalculateParticleMean();

void LogTrueState();
void LogParticles();
void LogMeanState();
void LogErrorCovariance();
void LogMeasurement(const Eigen::Vector2f& z);
void LogTime();
void LogWeights();

// Sets the location of the x an y position of the landmark in 
// vector x specified by landmark_id
void setLandmark(Eigen::VectorXf& x, int landmark_id, const Eigen::Vector2f& m);

// Returns the location of the x an y position of the landmark in 
// vector x specified by landmark_id
Eigen::Vector2f getLandmark(const Eigen::VectorXf& x, int landmark_id);

// Compute the measurement
// landmark_id: the landmarks id
Eigen::Vector2f getMeasurement(const Eigen::VectorXf& x, const Eigen::Vector2f& landmark, bool flag_true, bool flag_noise);

// The system function. 
// x: the state vector
// u: the input
// Ts: the time step
Eigen::VectorXf getg(const Eigen::VectorXf& x, const Eigen::VectorXf& u, float Ts);

// Compute the process noise covariance
// u: the input
Eigen::MatrixXf getM(const Eigen::VectorXf& u);

// Compute measurement jacobian H
// landmark_id: the landmarks id
Eigen::Matrix<float,2,2> getH(const Eigen::VectorXf& x, const Eigen::Vector2f& m);

// Propogate the true state vector in time
// u: the input
// Ts: the time step
void PropogateTrue(const Eigen::VectorXf& u, float Ts);

// The predict phase of the Fast Slam
// u: the input
// Ts: the time step
void Predict(const Eigen::VectorXf& u, float Ts);

// Checks to see if you are seeing a landmark for the first time
// z: the measurement to the landmark
// landmark_id: the landmarks id
bool NewLandmark(const Eigen::VectorXf& z, State& particle, int landmark_id);

// The update phase of the Fast Slam
// z: the measurement to the landmark
// landmark_id: the landmarks id
void Update(const Eigen::VectorXf& z, int landmark_id);

// Calculate the weight according to the measurment
float ImportanceFactor(const Eigen::Vector2f& err, const Eigen::Matrix<float,2,2> S);

void Resample();

// Normalize the weights of all of the particles so that the sum of the particle's 
// weights is one.
void NormalizeWeights();






// Environment Parameters
float vel_;
float radius_;
float fov_;
float angular_vel_;
int num_landmarks_;
int num_particles_;


Eigen::VectorXf x_;                // True states
Eigen::VectorXf xh_;               // The mean of the particles
Eigen::Matrix2f Q_;                // Measurement Covariance
std::vector<State> particles_;
std::vector<State> particles_tmp_;


// Used to add noise to measurements
std::default_random_engine gen_;
std::normal_distribution<double> randn_{0,1};
std::uniform_real_distribution<float> uniform_zero_one_{0, 1};

// Covariance coefficients
float alpha1_ = 0.1;
float alpha2_ = 0.01;
float alpha3_ = 0.01;
float alpha4_ = 0.1;

float sigma_r_= 0.1;
float sigma_phi_ = 0.05;

// Time variables
float t_;         // Current time
float Ts_;        // Time step
float tf_;        // Final time
float default_weight_ = 0.01;
float total_weight_ =1; // The sum of the weights of all of the particles

// Log Data
// std::ofstream log_K_;                 
std::ofstream log_t_;                
std::ofstream log_x_;  
std::ofstream log_particles_;          // Particles
// std::ofstream log_u_;  
std::ofstream log_P_; 
std::ofstream log_m_;           // Measurement log 
std::ofstream log_landmarks_;
std::ofstream log_weights_;     // The particle weights
std::ofstream log_xh_;        // The mean of the particles


};