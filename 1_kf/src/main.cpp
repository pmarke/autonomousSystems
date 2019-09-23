#include "kalman_filter.h"
#include <Eigen/Core>
#include <random>
#include <experimental/filesystem> 







int main(int argc, char **argv)
{
  // log data
  std::ofstream log_K;                 
  std::ofstream log_t;                
  std::ofstream log_x;  
  std::ofstream log_xh;  
  std::ofstream log_u;  
  std::ofstream log_P; 

  std::cout << "Writing log data to /tmp/AS_KalmanFilter" << std::endl;

    // Make sure the filepath exists
  if (!fs::exists("/tmp/AS_KalmanFilter"))
  {
    fs::create_directory("/tmp/AS_KalmanFilter");
  }

  log_K.open("/tmp/AS_KalmanFilter/k.log",std::ofstream::out);
  log_t.open("/tmp/AS_KalmanFilter/t.log",std::ofstream::out);
  log_x.open("/tmp/AS_KalmanFilter/x.log",std::ofstream::out);
  log_xh.open("/tmp/AS_KalmanFilter/xh.log",std::ofstream::out); 
  log_u.open("/tmp/AS_KalmanFilter/u.log",std::ofstream::out); 
  log_P.open("/tmp/AS_KalmanFilter/P.log",std::ofstream::out); 




  kf::KalmanFilter k(true);

  float m = 100;
  float b = 20;
  float Ts = 0.05;

  Eigen::Matrix<float,1,1> y;                 // Output
  Eigen::Matrix<float,1,1> u;                 // Input
  Eigen::Vector2f x{0,0};
  const Eigen::VectorXf* xh;
  Eigen::Matrix<float,2,2> Gc;                // Continous process noise matrix
  Gc << 1.0, 0.0,0.0,1.0;        
  const Eigen::MatrixXf* Gd;                  // Discrete process noise matrix
  Eigen::Matrix<float,2,1> Bc{0, 1.0f/m};     // Continous input matrix
  const Eigen::MatrixXf* Bd;                  // Discrete input matrix
  Eigen::Matrix<float,2,2> Ac;                // Continous system matrix
  Ac <<0, 1, 0, -b/m;   
  const Eigen::MatrixXf* Ad;                  // Discrete system matrix
  Eigen::Matrix<float,1,2> C {1,0};      

  Eigen::Matrix<float,2,2> Q;                 // Continous Process noise covariance
  Q << 0.0001,0,0,0.01;   
  const Eigen::MatrixXf* Qd;                  // Discrete Process noise covariance
  const Eigen::MatrixXf* K;                   // Kalman Gain

  Eigen::Matrix<float,1,1> R{0.001};             // Measurement noise covariance
  // Eigen::Matrix<float,1,1> R{0.001};             // Measurement noise covariance
  // Eigen::Matrix<float,2,2> P0{Eigen::Matrix<float,2,2>::Zero()};
  Eigen::Matrix<float,2,2> P0{Eigen::Matrix<float,2,2>::Identity()*0};
  const Eigen::MatrixXf* P;

  k.SetInitialEstimateState(x);
  k.SetObservationMatrix(C);
  k.SetSystemMatrixCont(Ac);
  k.SetInputMatrixCont(Bc);
  k.SetNoiseMatrixCont(Gc);
  k.SetR(R);
  k.SetQCont(Q);
  k.SetCovarianceMatrix(P0);
  k.Discretize(Ts);
  xh = k.GetEstimatedState();
  Gd = k.GetNoiseMatrixDiscrete();
  Bd = k.GetInputMatrixDiscrete();
  Ad = k.GetSystemMatrixDiscrete();
  Qd = k.GetProcessNoiseCovDiscrete();
  K = k.GetKalmanGain();
  P = k.GetErrorCovariance();

  std::default_random_engine generator;
  std::normal_distribution<double> q11(0,sqrtf(Q(0,0)));
  std::normal_distribution<double> q22(0,sqrt(Q(1,1)));
  std::normal_distribution<double> r(0,R(0,0));

  Eigen::Matrix<float,2,1> pNoise;
  Eigen::Matrix<float,1,1> mNoise;

  // Logging data
  std::ofstream log_K_t_;                 // Time
  std::ofstream log_K_m_;                 // Heading measurement and derivatives
  std::ofstream log_K_u_;                 // Camera's input
  std::ofstream log_K_estimate_;          // Estimated relative



  // std::cout << "Ac: " << std::endl << Ac <<std::endl;
  // std::cout << "Bc: " << std::endl << Bc <<std::endl;
  // std::cout << "Gc: " << std::endl << Gc <<std::endl;
  // std::cout << "C: " << std::endl << C <<std::endl;
  // std::cout << "Q: " << std::endl << Q <<std::endl;
  // std::cout << "R: " << std::endl << R <<std::endl;
  // std::cout << "P: " << std::endl << P <<std::endl;

  // std::cout << "xh: " << std::endl << *xh <<std::endl;
  // std::cout << "Gd: " << std::endl << *Gd <<std::endl;
  // std::cout << "Bd: " << std::endl << *Bd <<std::endl;
  // std::cout << "Ad: " << std::endl << *Ad <<std::endl;
  // std::cout << "Qd: " << std::endl << *Qd <<std::endl;
  k.SetQDiscrete(Q);


  // std::cout << "Qd: " << std::endl << *Qd <<std::endl;

  float T0 = 0;
  float Tf = 50;
  u <<0;

  for (float t=T0; t<Tf; t+=Ts)
  {


    // Get input
    if (t < 5)
      u << 50;
    else if( t>= 25 && t <30)
      u << -50;
    else
      u << 0;



    pNoise << q11(generator), q22(generator);
    mNoise << r(generator);


    // Dynaimes of real model
    x = (*Ad)*x+(*Bd)*u +Gc*pNoise;
    y = C*x + mNoise;


    // Estimate
    k.Predict(Ts,u);
    k.Update(y);


    // log data
    log_K.write( (char*) K->data(),  2*sizeof(float));                
    log_t.write( (char*) &t,           sizeof(float));                 
    log_x.write( (char*) &x,         2*sizeof(float));  
    log_xh.write( (char*) xh->data(),2*sizeof(float));  
    log_u.write( (char*) &u,           sizeof(float));   
    log_P.write( (char*) P->data(),  4*sizeof(float)); 


  }



  std::cout << "xh: " << std::endl << *xh <<std::endl;
  std::cout << "x: " << std::endl << x <<std::endl;

  log_K.close();                 
  log_t.close();                
  log_x.close();  
  log_xh.close();  
  log_u.close();  
  log_P.close(); 
  std::cout << "Finished writing log data!" << std::endl;

}