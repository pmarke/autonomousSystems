#include "simulation.h"




Sim::Sim(int num_landmarks, int M) : pf_(true,M), M_(M)
{


num_landmarks_=num_landmarks;
Sim::InitLog();
InitLandmarks();
InitSystem();

}

//---------------------------------------------------------------------------

Sim::~Sim()
{
  log_chi_.close();
  log_t_.close();                
  log_x_.close();  
  log_xh_.close();  
  log_u_.close();  
  log_m_.close();
  std::cout << "Finished writing log data!" << std::endl;
}

//---------------------------------------------------------------------------

void Sim::InitLog() {

  std::cout << "Writing log data to /tmp/AS_particle" << std::endl;

  // Make sure the filepath exists
  if (!fs::exists("/tmp/AS_particle"))
  {
    fs::create_directory("/tmp/AS_particle");
  }

  log_chi_.open("/tmp/AS_particle/chi.log",  std::ofstream::out);
  log_t_.open("/tmp/AS_particle/t.log",  std::ofstream::out);
  log_x_.open("/tmp/AS_particle/x.log",  std::ofstream::out);
  log_xh_.open("/tmp/AS_particle/xh.log",std::ofstream::out);
  log_u_.open("/tmp/AS_particle/u.log",  std::ofstream::out);
  log_m_.open("/tmp/AS_particle/m.log",  std::ofstream::out);

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

  Eigen::Vector4f x;    // Initial true states
  Eigen::Vector4f xh;   // Initial estimated state

  x << -5, -3, kPI/2,0;
//  xh.setZero();
  // xh = x;


  pf_.SetTrueState(x);
  u_ = pf_.GetInput();
  x_ = pf_.GetTrueState();
  xh_= pf_.GetEstimateState();
  z_ = pf_.GetTrueMeasurement();
//  zh_= pf_.GetEstimateMeasurement();
  chi_ = pf_.GetParticles();
  pf_.SetSystemFunctionPointer(std::bind(&Sim::SystemFunction, this, _1, _2,_3,_4));
  pf_.SetMeasurementFunctionPointer(std::bind(&Sim::MeasFunction, this, _1, _2,_3));


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

  Eigen::Vector4f x_delta; // change in true state
  x_delta << -v/w*sin(th) + v/w*sin(th+w*Ts), 
            v/w*cos(th) - v/w*cos(th+w*Ts),
            w*Ts,0;

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

  Eigen::Vector2f u(0,0);   // input
  Eigen::Matrix2f Q;   // process noise
  pf_.SetInput(u);
  LogData();





  while (t_ < tf_)
  {

    GetInputAndProcessCovariance(u, Q, t_);
    pf_.SetInput(u);
    // LogData();
    pf_.Predict(Ts_);

    for (unsigned int i=0; i < num_landmarks_; i++)
    {
      id_ = i;
      // std::cout << "id: " << id_ << std::endl;

      pf_.Update();
//      LogMeasurement();

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

//    std::cout << "P: " << std::endl << *P_ << std::endl;
//    std::cout << "P eig: " << std::endl << P_->eigenvalues() << std::endl;
//
    std::cout << "x: " << std::endl << *x_ << std::endl;
    std::cout << "xh: " << std::endl << *xh_ << std::endl;

}

//---------------------------------------------------------------------------

void Sim::LogData()
{
    // log data
    for (unsigned int i=0; i < M_; i++) {
//      std::cerr << "chi: " << std::endl << chi_[i] << std::endl;

      log_chi_.write( (char*) chi_[i].data(),   4*sizeof(float));
//      std::cerr << "chi: " << std::endl << chi_[i] << std::endl;
    }

    log_t_.write( (char*) &t_,          sizeof(float));                 
    log_x_.write( (char*) x_->data(),  3*sizeof(float));  
    log_xh_.write( (char*) xh_->data(), 3*sizeof(float));  
    log_u_.write( (char*) u_->data(),   2*sizeof(float));   
//    log_P_.write( (char*) P_->data(),   9*sizeof(float));
}

//---------------------------------------------------------------------------

//void Sim::LogMeasurement()
//{
//  log_m_.write( (char*) z_->data(),   2*sizeof(float));
//}