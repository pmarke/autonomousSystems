#include "EKF.h"


EKF::EKF(int num_landmarks)
{

num_landmarks_=num_landmarks;

std::cout << "Writing log data to /tmp/AS_EKF" << std::endl;

  // Make sure the filepath exists
if (!fs::exists("/tmp/AS_EKF"))
{
  fs::create_directory("/tmp/AS_EKF");
}

log_K_.open("/tmp/AS_EKF/k.log",std::ofstream::out);
log_t_.open("/tmp/AS_EKF/t.log",std::ofstream::out);
log_x_.open("/tmp/AS_EKF/x.log",std::ofstream::out);
log_xh_.open("/tmp/AS_EKF/xh.log",std::ofstream::out); 
log_u_.open("/tmp/AS_EKF/u.log",std::ofstream::out); 
log_P_.open("/tmp/AS_EKF/P.log",std::ofstream::out); 
log_m_.open("/tmp/AS_EKF/m.log",std::ofstream::out);

log_m_.write( (char*) &num_landmarks_,   sizeof(int));
// float num = (float) num_landmarks;

// Initialize landmarks
InitLandmarks();

// Initialize error covariance
P_.setIdentity();
K_.setZero();

// Init meas covariance
Q_ << powf(sigma_r_,2), 0, 0, powf(sigma_phi_,2);

// Initialize true state
x_ << -5, -3, kPI/2;
xh_.setZero();
// xh_<<100000,-100,kPI/2;
u_.setZero();

t_ = 0;
Ts_ = 0.1;
tf_ = 20;



LogData();


}

//---------------------------------------------------------------------------

EKF::~EKF()
{
  log_K_.close();                 
  log_t_.close();                
  log_x_.close();  
  log_xh_.close();  
  log_u_.close();  
  log_P_.close(); 
  log_m_.close();
  std::cout << "Finished writing log data!" << std::endl;
}


//---------------------------------------------------------------------------


void EKF::InitLandmarks()
{
  if( num_landmarks_ > 0){
      landmarks_.push_back(Eigen::Vector2f(6.0,4.0));
      Eigen::Vector2f v(6.0,4.0);
      log_m_.write( (char*) v.data(),   2*sizeof(float));
    }
  if( num_landmarks_ > 1) {
    landmarks_.push_back(Eigen::Vector2f(-7.0,8.0));
    log_m_.write( (char*) landmarks_[1].data(),   2*sizeof(float));
  }
  if(num_landmarks_ > 2) {
    landmarks_.push_back(Eigen::Vector2f(6.0,-4.0));
    log_m_.write( (char*) landmarks_[2].data(),   2*sizeof(float));
  }


  std::uniform_int_distribution<int> dist(-10,10);
  Eigen::Vector2f new_landmark;
  int i = 3;
  while (i < num_landmarks_)
  {
    new_landmark << dist(gen_), dist(gen_);
    bool landmark_exists = false;

    // see if landmark already exists
    for (int j = 0; j < landmarks_.size(); j++)
    {
      if (new_landmark == landmarks_[j])
      {
        landmark_exists = true;
      }
    }

    if (!landmark_exists)
    {
      landmarks_.push_back(new_landmark);
      log_m_.write( (char*) landmarks_[i].data(),   2*sizeof(float));
      i++;
    }

  }


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
  
  LogMeasurement(z);

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

void EKF::LogData()
{
    // log data
    log_K_.write( (char*) K_.data(),   6*sizeof(float));                
    log_t_.write( (char*) &t_,          sizeof(float));                 
    log_x_.write( (char*) x_.data(),  3*sizeof(float));  
    log_xh_.write( (char*) xh_.data(), 3*sizeof(float));  
    log_u_.write( (char*) u_.data(),   2*sizeof(float));   
    log_P_.write( (char*) P_.data(),   9*sizeof(float)); 
}

//---------------------------------------------------------------------------

void EKF::LogMeasurement(const Eigen::Vector2f& z)
{
  log_m_.write( (char*) z.data(),   2*sizeof(float));
}

//---------------------------------------------------------------------------

void EKF::sim()
{
  Eigen::Vector2f u;

  while( t_ < tf_)
  {

    // std::cout << "here1" << std::endl;
    u = GetInput(t_);
    u_ = u;
    // std::cout << "here2" << std::endl;

    Step(u,t_,Ts_);
    // std::cout << "here3" << std::endl;

    Predict(u, Ts_);
    // std::cout << "here4" << std::endl;


    for (int i = 0; i < num_landmarks_; i++)
    {
      Update(i);
    }
    // std::cout << "here4" << std::endl;

    


    t_ += Ts_;

    LogData();
  }



  std::cout << "x: " << std::endl << x_ << std::endl;
  std::cout << "xh: " << std::endl << xh_ << std::endl;
  std::cout << "P: " << std::endl << P_ << std::endl;


}