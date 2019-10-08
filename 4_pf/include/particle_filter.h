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
#include <algorithm>

namespace fs = std::experimental::filesystem;

typedef std::function<Eigen::VectorXf(const Eigen::VectorXf&  x, const Eigen::VectorXf& u, float Ts, bool addNoise)> SysFunPtr;
typedef std::function<Eigen::VectorXf(const Eigen::VectorXf&  x, const Eigen::VectorXf& u, bool addNoise)> MeasFunPtr;


const float kPI = 3.14159;

class ParticleFilter {

public:

explicit ParticleFilter(bool propogateTrue, unsigned int M);
~ParticleFilter();



    void SetInput(const Eigen::VectorXf& u);
    void SetTrueState(const Eigen::VectorXf& x);
//    void SetEstimateState(const Eigen::VectorXf& xh);
    void SetTrueMeasurement(const Eigen::VectorXf& z);
//    void SetEstimateMeasurement(const Eigen::VectorXf& zh);

    // Set function pointers
    void SetSystemFunctionPointer(SysFunPtr f);
    void SetMeasurementFunctionPointer(MeasFunPtr g);
    void SetLandmarkLocationPointer(std::function<Eigen::VectorXf()> l_);

    const Eigen::VectorXf* GetInput();
    const Eigen::VectorXf* GetTrueState();
    const Eigen::Vector3f* GetEstimateState();
    const Eigen::VectorXf* GetTrueMeasurement();
//    const Eigen::VectorXf* GetEstimateMeasurement();
    const Eigen::VectorXf* GetParticles();

    void Predict(float Ts);
    void Update();


private:

    void InitializeEstimate();
    void Resample();
    float GetWeight(Eigen::Vector2f z, Eigen::Vector2f zh);
    Eigen::Vector4f GetParticleFromMeas(Eigen::Vector2f zh);

    bool propogateTrue_;
    unsigned int M_;



    Eigen::VectorXf u_;    // Input to system
    Eigen::VectorXf x_;    // True State
    Eigen::Vector3f xh_;    // True State
    Eigen::VectorXf* chi_=nullptr;
    Eigen::VectorXf* chib_=nullptr;
    Eigen::VectorXf z_;    // True Measurement
    Eigen::VectorXf* zh_;   // Estimated Measurement
    Eigen::Matrix2f R_;
    float ExpConst_;
    float t_=0;

    float alpha_slow_=0.01;
    float alpha_fast = 0.1;
    float w_slow_ = 0;
    float w_fast_ = 0;
    float w_t_ = 0;

    // Pointer to the system function
    // x is the state of the system
    // u is the input to the system
    SysFunPtr f_;

    // Pointer to the measurement function
    MeasFunPtr g_;

    std::function<Eigen::VectorXf()> GetLandmarkLocation_;

    std::default_random_engine gen_;
    std::uniform_real_distribution<float> uniform_x_{-20,20};
    std::uniform_real_distribution<float> uniform_y_{-20,20};
    std::uniform_real_distribution<float> uniform_th_{0,2*kPI};
    std::uniform_real_distribution<float> uniform_zero_one_{0, 1};

};
