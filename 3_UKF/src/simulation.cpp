#include "simulation.h"




Sim::Sim(int num_landmarks) : ukf_(true)
{


num_landmarks_=num_landmarks;
Sim::InitLog();
InitLandmarks();
InitSystem();

}

//---------------------------------------------------------------------------

Sim::~Sim()
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

void Sim::InitLog() {

  std::cout << "Writing log data to /tmp/AS_UKF" << std::endl;

  // Make sure the filepath exists
  if (!fs::exists("/tmp/AS_UKF"))
  {
    fs::create_directory("/tmp/AS_UKF");
  }

  log_K_.open("/tmp/AS_UKF/k.log",  std::ofstream::out);
  log_t_.open("/tmp/AS_UKF/t.log",  std::ofstream::out);
  log_x_.open("/tmp/AS_UKF/x.log",  std::ofstream::out);
  log_xh_.open("/tmp/AS_UKF/xh.log",std::ofstream::out); 
  log_u_.open("/tmp/AS_UKF/u.log",  std::ofstream::out); 
  log_P_.open("/tmp/AS_UKF/P.log",  std::ofstream::out); 
  log_m_.open("/tmp/AS_UKF/m.log",  std::ofstream::out);

  log_m_.write( (char*) &num_landmarks_,   sizeof(int));
}

//---------------------------------------------------------------------------

void Sim::InitLandmarks() {

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

void Sim::InitSystem() {

  using namespace std::placeholders;

  Eigen::Matrix2f R;    // Measurement noise
  Eigen::Matrix3f P0;   // Initial error covariance
  Eigen::Vector3f x;    // Initial true states
  Eigen::Vector3f xh;   // Initial estimated state

  R << powf(sigma_r_,2), 0, 0, powf(sigma_phi_,2);
  P0.setIdentity();
  x << -5, -3, kPI/2;
  xh.setZero();
  // xh = x;


  ukf_.SetMeasurementCovariance(R);
  ukf_.SetErrorCovariance(P0);
  ukf_.SetTrueState(x);
  ukf_.SetEstimateState(xh);

  R_ = ukf_.GetMeasurementCovariance();
  Q_ = ukf_.GetProcessCovariance();
  P_ = ukf_.GetErrorCovariance();
  K_ = ukf_.GetKalmanGain();
  u_ = ukf_.GetInput();
  x_ = ukf_.GetTrueState();
  xh_= ukf_.GetEstimateState();
  z_ = ukf_.GetTrueMeasurement();
  zh_= ukf_.GetEstimateMeasurement();
  ukf_.SetSystemFunctionPointer(std::bind(&Sim::SystemFunction, this, _1, _2,_3,_4));
  ukf_.SetMeasurementFunctionPointer(std::bind(&Sim::MeasFunction, this, _1, _2,_3));
  ukf_.SetWeightParameters(0.15,50,2);


}

//---------------------------------------------------------------------------

Eigen::VectorXf Sim::SystemFunction(const Eigen::VectorXf&  x, const Eigen::VectorXf& u, float Ts, bool addNoise) {

  // Get Velocity, Angular Velocity, and theta
  float v = u(0);       
  float w = u(1);
  float th = x(2);

  Ts = Ts_;

  if (addNoise) {
    v += sqrt((alpha1_*powf(v,2)+alpha2_*powf(w,2)))*randn_(gen_);
    w += sqrt((alpha3_*powf(v,2)+alpha4_*powf(w,2)))*randn_(gen_);
  }

  Eigen::Vector3f x_delta; // change in true state
  x_delta << -v/w*sin(th) + v/w*sin(th+w*Ts), 
            v/w*cos(th) - v/w*cos(th+w*Ts),
            w*Ts;

  return x + x_delta;

}

//---------------------------------------------------------------------------

Eigen::VectorXf Sim::MeasFunction(const Eigen::VectorXf&  x, const Eigen::VectorXf& u, bool addNoise) {

  Eigen::Vector2f z; // Measurement 

  float mx = landmarks_[id_](0);  // The x position of the landmark seen
  float my = landmarks_[id_](1);  // The y position of the landmark seen

  float px = x(0); // X position estimate
  float py = x(1); // Y position estimate
  float th = x(2); // Theta estimate

  float q = powf(mx-px,2)+powf(my-py,2);    // Range^2 to landmark

  if (addNoise)
    z << sqrt(q) + sigma_r_*randn_(gen_),atan2(my-py,mx-px)+sigma_phi_*randn_(gen_)-th;
  else
    z << sqrt(q) ,atan2(my-py,mx-px)-th;

  // std::cout << "id: " << id_ << std::endl;

  return z;
}

//---------------------------------------------------------------------------

void Sim::GetInputAndProcessCovariance(Eigen::Vector2f& u, Eigen::Matrix2f& Q, float t) {

  // Compute velocity and angular velocity
  float v = 1+0.5*cos(2*kPI*0.2*t);
  float w = -0.2 +2*cos(2*kPI*0.6*t);
  // std::cout << "w: " << w << std::endl;

  Q << alpha1_*powf(v,2)+alpha2_*powf(w,2),              0,
               0,                            alpha3_*powf(v,2)+alpha4_*powf(w,2); 

  u << v,w;
}

//---------------------------------------------------------------------------


void Sim::Simulate()
{

  Eigen::Vector2f u;   // input
  Eigen::Matrix2f Q;   // process noise



  while (t_ < tf_)
  {

    GetInputAndProcessCovariance(u, Q, t_);
    ukf_.SetProcessCovariance(Q);
    ukf_.SetInput(u);
    // LogData();
    ukf_.Predict(Ts_);

    for (unsigned int i=0; i < num_landmarks_; i++)
    {
      id_ = i;
      // std::cout << "id: " << id_ << std::endl;

      ukf_.Update();  
      LogMeasurement();   

    }


    // std::cout << "Q: " << std::endl << *Q_ << std::endl;
    // std::cout << "R: " << std::endl << *R_ << std::endl;
    // std::cout << "P: " << std::endl << *P_ << std::endl;
    // std::cout << "K: " << std::endl << *K_ << std::endl;
    // std::cout << "u: " << std::endl << *u_ << std::endl;
    // std::cout << "x: " << std::endl << *x_ << std::endl;
    // std::cout << "xh: " << std::endl << *xh_ << std::endl;
    // std::cout << "z: " << std::endl << *z_ << std::endl;
    // std::cout << "zh: " << std::endl << *zh_ << std::endl;


    t_+=Ts_;

    LogData();

  }

    std::cout << "P: " << std::endl << *P_ << std::endl;
    std::cout << "P eig: " << std::endl << P_->eigenvalues() << std::endl;

    std::cout << "x: " << std::endl << *x_ << std::endl;
    std::cout << "xh: " << std::endl << *xh_ << std::endl;

}

//---------------------------------------------------------------------------

void Sim::LogData()
{
    // log data
    log_K_.write( (char*) K_->data(),   6*sizeof(float));                
    log_t_.write( (char*) &t_,          sizeof(float));                 
    log_x_.write( (char*) x_->data(),  3*sizeof(float));  
    log_xh_.write( (char*) xh_->data(), 3*sizeof(float));  
    log_u_.write( (char*) u_->data(),   2*sizeof(float));   
    log_P_.write( (char*) P_->data(),   9*sizeof(float)); 
}

//---------------------------------------------------------------------------

void Sim::LogMeasurement()
{
  log_m_.write( (char*) z_->data(),   2*sizeof(float));
}