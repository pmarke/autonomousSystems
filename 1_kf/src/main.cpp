#include "kalman_filter.h"
#include <Eigen/Core>








int main(int argc, char **argv)
{
 
  kf::KalmanFilter k(true);

  float m = 100;
  float b = 20;
  float Ts = 0.05;

  Eigen::Vector2f x{0,0};
  const Eigen::Vector2f* xh;
  Eigen::Matrix<float,2,2> Gc;                   // Continous process noise matrix
  Gc << 1.0, 0.0,0.0,1.0;        
  const Eigen::Matrix<float,2,2>* Gd;                  // Discrete process noise matrix
  Eigen::Matrix<float,2,1> Bc{0, 1.0f/m};        // Continous input matrix
  const Eigen::Matrix<float,2,1>* Bd;                  // Discrete input matrix
  Eigen::Matrix<float,2,2> Ac;                   // Continous system matrix
  Ac <<0, 1, 0, -b/m;   
  const Eigen::Matrix<float,2,2>* Ad;                  // Discrete system matrix
  Eigen::Matrix<float,1,2> C {1,0};      

  Eigen::Matrix<float,2,2> Q;                    // Continous Process noise covariance
  Q << 0.0001,0,0,0.01;   
  const Eigen::Matrix<float,2,2>* Qd;                  // Discrete Process noise covariance

  Eigen::Matrix<float,1,1> R{0.001};             // Measurement noise covariance
  Eigen::Matrix<float,2,2> P{Eigen::Matrix<float,2,2>::Zero()};

  k.SetInitialEstimateState(x);
  k.SetObservationMatrix(C);
  k.SetSystemMatrixCont(Ac);
  k.SetInputMatrixCont(Bc);
  k.SetNoiseMatrixCont(Gc);
  k.SetR(R);
  k.SetQCont(Q);
  k.SetCovarianceMatrix(P);
  k.Discretize(Ts);
  xh = k.GetEstimatedState();
  Gd = k.GetNoiseMatrixDiscrete();
  Bd = k.GetInputMatrixDiscrete();
  Ad = k.GetSystemMatrixDiscrete();
  Qd = k.GetProcessNoiseCovDiscrete();



  std::cout << "Ac: " << std::endl << Ac <<std::endl;
  std::cout << "Bc: " << std::endl << Bc <<std::endl;
  std::cout << "Gc: " << std::endl << Gc <<std::endl;
  std::cout << "C: " << std::endl << C <<std::endl;
  std::cout << "Q: " << std::endl << Q <<std::endl;
  std::cout << "R: " << std::endl << R <<std::endl;
  std::cout << "P: " << std::endl << P <<std::endl;






}