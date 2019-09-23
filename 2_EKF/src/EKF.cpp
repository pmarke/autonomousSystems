#include "EKF.h"


EKF::EKF()
{

// Initialize landmarks
landmarks_.push_back(Eigen::Vector2f(6,4));
landmarks_.push_back(Eigen::Vector2f(-7,8));
landmarks_.push_back(Eigen::Vector2f(6,-4));

// Initialize error covariance
P_.setIdentity()*10000;

// Init meas covariance
Q_ << powf(sigma_r_,2), 0, 0, powf(sigma_phi_,2);

// Initialize true state
x_ << -5, -3, kPI/2;
xh_ = x_*100000;

t_ = 0;
Ts_ = 0.1;
tf_ = 20;

}


//---------------------------------------------------------------------------

void EKF::Predict(const Eigen::Vector2f& u, float Ts)
{
  float v = u(0);   // Velocity input
  float w = u(1);   // Angular velocity input

  float px = xh_(0); // X position estimate
  float py = xh_(1); // Y position estimate
  float th = xh_(2); // Theta estimate

  Eigen::Matrix3f G;          // Jacobian
  Eigen::Matrix<float,3,2> V; // Noise to system map
  Eigen::Matrix2f M;          // Noise covariance
  Eigen::Vector3f mu_delta;   // The change in mean

  G << 1,    0,  -v/w*cos(th)+v/w*cos(th+w*Ts),
       0,    1,  -v/w*sin(th)+v/w*sin(th+w*Ts),
       0,    0,               1; 

  V << (-sin(th)+sin(th+w*Ts))/w, v*(sin(th)-sin(th+w*Ts))/powf(w,2)+ v*cos(th+w*Ts)*Ts/w,
       (cos(th)-cos(th+w*Ts))/w, -v*(cos(th)-cos(th+w*Ts))/powf(w,2)+ v*sin(th+w*Ts)*Ts/w,
        0, Ts;

  M << alpha1_*powf(v,2)+alpha2_*powf(w,2),              0,
                   0,                            alpha3_*powf(v,2)+alpha4_*powf(w,2);     

  mu_delta << -v/w*sin(th) + v/w*sin(th+w*Ts), 
               v/w*cos(th) - v/w*cos(th+w*Ts),
               w*Ts;

  xh_ += mu_delta;

  P_ = G*P_*G.transpose() + V*M*V.transpose();
}

//---------------------------------------------------------------------------

void EKF::Update(const int id)
{

  Eigen::Vector2f z = GetMeasurement(id);

  Eigen::Vector2f zh;         // Estimated measurement 
  Eigen::Matrix<float,2,3> H; // Linearized observation function

  float mx = landmarks_[id](0);  // The x position of the landmark seen
  float my = landmarks_[id](1);  // The y position of the landmark seen

  float px = xh_(0); // X position estimate
  float py = xh_(1); // Y position estimate
  float th = xh_(2); // Theta estimate

  float q = powf(mx-px,2)+powf(my-py,2);    // Range^2 to landmark

  zh << sqrt(q),atan2(my-py,mx-px)-th;
  H << -(mx-px)/sqrt(q), -(my-py)/sqrt(q), 0,
        (my-py)/q,        (mx-px)/q,  -1;    

  S_ = H*P_*H.transpose() + Q_;
  K_ = P_*H.transpose()*S_.inverse();
  xh_ += K_*(z-zh);
  P_ = (Eigen::Matrix3f::Identity()-K_*H)*P_;
}


//---------------------------------------------------------------------------

void EKF::Step(const Eigen::Vector2f& u, float t, float Ts)
{
  // Compute velocity and angular velocity
  float v = u(0);
  float w = u(1);
  float th = x_(2);

  // std::cout << "in 1" << std::endl;

  // Add process noise
  v += sqrt((alpha1_*powf(v,2)+alpha2_*powf(w,2)))*randn_(gen_);
  w += sqrt((alpha3_*powf(v,2)+alpha4_*powf(w,2)))*randn_(gen_);
  // std::cout << sqrt((alpha3_*powf(v,2)+alpha4_*powf(w,2))) <<std::endl << std::endl;
  // std::cout << randn_(gen_) <<std::endl << std::endl;

  // std::cout << "in 2" << std::endl;


  Eigen::Vector3f x_delta; // change in true state

  x_delta << -v/w*sin(th) + v/w*sin(th+w*Ts), 
              v/w*cos(th) - v/w*cos(th+w*Ts),
              w*Ts;

  // std::cout << "in 3" << std::endl;


  x_ += x_delta;

}

//---------------------------------------------------------------------------

Eigen::Vector2f EKF::GetMeasurement(int id)
{
  Eigen::Vector2f z; // Measurement 

  float mx = landmarks_[id](0);  // The x position of the landmark seen
  float my = landmarks_[id](1);  // The y position of the landmark seen

  float px = x_(0); // X position estimate
  float py = x_(1); // Y position estimate
  float th = x_(2); // Theta estimate

  float q = powf(mx-px,2)+powf(my-py,2);    // Range^2 to landmark

  z << sqrt(q) + sigma_r_*randn_(gen_),atan2(my-py,mx-px)+sigma_phi_*randn_(gen_)-th;
  return z;
}
//---------------------------------------------------------------------------

Eigen::Vector2f EKF::GetInput(float t)
{
  Eigen::Vector2f u;
    // Compute velocity and angular velocity
  float v = 1+0.5*cos(2*kPI*0.2*t);
  float w = -0.2 +2*cos(2*kPI*0.6*t);

  u << v,w;
  return u;
}

//---------------------------------------------------------------------------

void EKF::sim()
{
  Eigen::Vector2f u;

  while( t_ < tf_)
  {

    // std::cout << "here1" << std::endl;
    u = GetInput(t_);
    // std::cout << "here2" << std::endl;

    Step(u,t_,Ts_);
    // std::cout << "here3" << std::endl;

    Predict(u, Ts_);
    // std::cout << "here4" << std::endl;


    for (int i = 0; i < 3; i++)
    {
      Update(i);
    }
    // std::cout << "here4" << std::endl;


    t_ += Ts_;
  }

  std::cout << "x: " << std::endl << x_ << std::endl;
  std::cout << "xh: " << std::endl << xh_ << std::endl;
  std::cout << "P: " << std::endl << P_ << std::endl;


}