#include "fastSlam/fast_slam.h"



FastSlam::FastSlam(int num_landmarks, float fov, int num_particles, float vel, float radius) {

vel_ = vel;
fov_ = fov;
radius_ = radius;
angular_vel_ = vel_/radius_;
if (num_landmarks < 1)
  num_landmarks = 1;
num_landmarks_ = num_landmarks;
num_particles_ = num_particles;

InitLog();
std::cerr << "init log: " << std::endl;
InitEnvrionment();
std::cerr << "init env: " << std::endl;

InitParticles();
std::cerr << "init particles: " << std::endl;


}

//---------------------------------------------------------------------------

FastSlam::~FastSlam() {

}

//---------------------------------------------------------------------------

void FastSlam::InitLog() {
  // Initilize logging data
  std::cout << "Writting log data to /home/mark/projects/autonomousSystems/7_FAST_SLAM/log" << std::endl;
    
  // Make sure the filepath exists
  if (!fs::exists("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log"))
  {
    fs::create_directory("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log");
  }

  // log_K_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/k.log",std::ofstream::out);
  log_t_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/t.log",std::ofstream::out);
  log_x_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/x.log",std::ofstream::out);
  log_xh_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/xh.log",std::ofstream::out); 
  // log_u_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/u.log",std::ofstream::out); 
  log_P_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/P.log",std::ofstream::out);
  // log_P_full_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/P_full.log",std::ofstream::out);
  log_m_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/m.log",std::ofstream::out);
  log_landmarks_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/landmarks.log",std::ofstream::out);
  log_weights_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/weights.log",std::ofstream::out);
  log_particles_.open("/home/mark/projects/autonomousSystems/7_FAST_SLAM/log/particles.log",std::ofstream::out);
  log_landmarks_.write( (char*) &num_landmarks_,   sizeof(int));
  log_particles_.write( (char*) &num_particles_,   sizeof(int));
}

//---------------------------------------------------------------------------

void FastSlam::CloseLogs() {

  log_t_.close();                
  log_x_.close();  
  log_xh_.close();  
  // log_u_.close();  
  log_P_.close(); 
  // log_P_full_.close();
  log_m_.close();
  log_weights_.close();
  log_particles_.close();
  log_landmarks_.close();
  std::cout << "Finished writing log data!" << std::endl;

}

//---------------------------------------------------------------------------

void FastSlam::InitEnvrionment() {

t_ = 0.0f;
Ts_ = 0.1f;
tf_ = 2.0f*kPI*radius_/vel_*2.0f;  // approximate time for one orbit

// Initialize the true and estimated states
x_ = Eigen::MatrixXf::Zero(3+2*num_landmarks_,1);
x_(0) = 0.0f;
x_(1) = 0.0f;
x_(2) = kPI/2.0f;

// Create Landmarks
Eigen::Vector2f m;
Eigen::Vector2f z;
z << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN();
m << x_(0), x_(1)+radius_;
setLandmark(x_,0,m);
LogMeasurement(z);
log_landmarks_.write( (char*) m.data(),   sizeof(float)*2);

float w_sign = 1;
if (angular_vel_ > 0)
  w_sign = -1;

if (num_landmarks_ > 1) {
  float delta_th = 2.0f*kPI/(num_landmarks_);
  float curr_th = delta_th;
  Eigen::Vector2f center(x_(0)+radius_*w_sign,x_(1));
  for (unsigned int ii = 1; ii < num_landmarks_; ii++) {
    m << center(0)-radius_*cos(curr_th)*w_sign, center(1) + radius_*sin(curr_th);
    curr_th += delta_th;
    setLandmark(x_,ii,m);
    LogMeasurement(z);
    log_landmarks_.write( (char*) m.data(),   sizeof(float)*2);
  }
}

}

//---------------------------------------------------------------------------

void FastSlam::InitParticles() {

  Eigen::Vector3f xh(0,0,kPI/2.0f);
  //   std::cerr << "par s1: "<< std::endl;

  //   State s1(xh,num_landmarks_,num_particles_,1);
  
  //   std::cerr << "par s2: "<< std::endl;

  // State s2(xh,num_landmarks_,num_particles_,2);
  //   std::cerr << "par s3: "<< std::endl;
  // State s3(xh,num_landmarks_,num_particles_,3);
  //   std::cerr << "par s4: "<< std::endl;
  // State s4(xh,num_landmarks_,num_particles_,4);
  //   std::cerr << "par s5: "<< std::endl;
  // State s5(xh,num_landmarks_,num_particles_,5);
  //   std::cerr << "par s6: "<< std::endl;
  // State s6(xh,num_landmarks_,num_particles_,6);
  //   std::cerr << "par s7: "<< std::endl;
  // State s7(xh,num_landmarks_,num_particles_,7);
  //   std::cerr << "par s8: "<< std::endl;
  // State s8(xh,num_landmarks_,num_particles_,8);
  //   std::cerr << "par s9: "<< std::endl;
  // State s9(xh,num_landmarks_,num_particles_,9);

  // std::cerr << "num particles: " << num_particles_<< std::endl;
  // std::cerr << "num landmarks: " << num_landmarks_<< std::endl;
  


  for (unsigned int k=0; k < num_particles_; k++) {
    // particles_.push_back(s);
    State s(xh,num_landmarks_,num_particles_);
    // std::cerr << "par k: " << k << std::endl;

    particles_.push_back(s);
      // std::cerr << "states: " << std::endl << particles_[k].stacked_states_ << std::endl;
  // std::cerr << "P: " << std::endl << particles_[k].stacked_covariance_ << std::endl;

  }

  Q_ << powf(sigma_r_,2), 0, 0, powf(sigma_phi_,2);
  // std::cerr << "here " << particles_.size() << std::endl;

  CalculateParticleMean();

  Eigen::Matrix2f P;
  Eigen::Vector2f m;
  P << 1,2,3,4;
  m << 5,6;

  particles_[0].landmarks_[0] = m;
  particles_[0].P_[0] = P;

  // std::cerr << "first mem: " << &particles_[0].P_[0] << std::endl;

  std::cerr << std::endl << std::endl << std::endl;


  // for (unsigned int k=0; k < num_particles_; k++) {
  //   // particles_.push_back(s);
  //   // particles_.emplace_back(xh,num_landmarks_,num_particles_,k);
  //   // std::cerr << "par k: " << k << std::endl;
  //   std::cerr << "states: " << std::endl << particles_[k].stacked_states_ << std::endl;
  //   std::cerr << "P: " << std::endl << particles_[k].stacked_covariance_ << std::endl;
  //   // std::cerr << "mem states: " << std::endl << &particles_[k].stacked_states_ << std::endl;
  //   // std::cerr << "mem P: " << std::endl << &particles_[k].stacked_covariance_ << std::endl;

  // }
}

//---------------------------------------------------------------------------

void FastSlam::CalculateParticleMean() {
  xh_ = Eigen::VectorXf::Zero(3+2*num_landmarks_);

  for (unsigned int k=0; k < num_particles_; k++) {
    xh_ += particles_[k].stacked_states_*particles_[k].weight_;
  }
}

//---------------------------------------------------------------------------

void FastSlam::LogTrueState() {

  log_x_.write( (char*) x_.data(),  3*sizeof(float));
}

//---------------------------------------------------------------------------

void FastSlam::LogParticles() {
  for (unsigned int k=0; k < num_particles_; k++) {
    log_particles_.write( (char*) particles_[k].stacked_states_.data(),  (3+2*num_landmarks_)*sizeof(float));

  }  
}

//---------------------------------------------------------------------------

void FastSlam::LogMeanState() {

  log_xh_.write( (char*) xh_.data(),  (3+2*num_landmarks_)*sizeof(float));
}

//---------------------------------------------------------------------------

void FastSlam::LogErrorCovariance() {

  for (unsigned int k=0; k < num_particles_; k++) {
    
    log_particles_.write( (char*) particles_[k].stacked_covariance_.data(),  (4*num_landmarks_)*sizeof(float));

  }  
}

//---------------------------------------------------------------------------

void FastSlam::LogMeasurement(const Eigen::Vector2f& z) {

  log_m_.write( (char*) z.data(),   2*sizeof(float));
}

//---------------------------------------------------------------------------

void FastSlam::LogTime() {

  log_t_.write( (char*) &t_,          sizeof(float)); 
}

//---------------------------------------------------------------------------

void FastSlam::setLandmark(Eigen::VectorXf& x, int landmark_id, const Eigen::Vector2f& m) {
 
  x(3+2*landmark_id) = m(0);
  x(4+2*landmark_id) = m(1);
}

//---------------------------------------------------------------------------

Eigen::Vector2f FastSlam::getLandmark(const Eigen::VectorXf& x, int landmark_id) {

  Eigen::Vector2f landmark; // Measurement 

  float mx = x(3+2*landmark_id);  // The x position of the landmark seen
  float my = x(4+2*landmark_id);  // The y position of the landmark seen

  landmark << mx, my;

  return landmark;
}

//---------------------------------------------------------------------------

Eigen::Vector2f FastSlam::getMeasurement(const Eigen::VectorXf& x, const Eigen::Vector2f& landmark, bool flag_true, bool flag_noise) {

  Eigen::Vector2f z; // Measurement 

  // std::cerr << "m: " << std::endl << m << std::endl;
  // std::cerr << "landmark_id: " << std::endl << landmark_id << std::endl;

  

  float mx = landmark(0);  // The x position of the landmark seen
  float my = landmark(1);  // The y position of the landmark seen

  float px = x(0); // X position estimate
  float py = x(1); // Y position estimate
  float th = x(2); // Theta estimate

  float q = powf(mx-px,2)+powf(my-py,2);    // Range^2 to landmark

  if (flag_noise)
    z << sqrt(q) + sigma_r_*randn_(gen_),atan2(my-py,mx-px)+sigma_phi_*randn_(gen_)-th;
  else
    z << sqrt(q),atan2(my-py,mx-px)-th;

  // std::cout << "zb: " << z(1) << std::endl;


  while (z(1) > kPI)
    z(1) = z(1)-2*kPI;
  while(z(1)<-kPI)
    z(1) = z(1) + 2*kPI;

  // std::cout << "z: " << z(1) << std::endl;

  // See if the measurement is in the field of view if
  // it is associated with the true state
  // if it isn't in the field of view. Set it to Nan
  if( fabs(z(1))>fov_ && flag_true)
    z << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN();

  return z;
}

//---------------------------------------------------------------------------


Eigen::VectorXf FastSlam::getg(const Eigen::VectorXf& x, const Eigen::VectorXf& u, float Ts) {

  float v = u(0);   // Velocity input
  float w = u(1);   // Angular velocity input

  float px = x(0); // X position estimate
  float py = x(1); // Y position estimate
  float th = x(2); // Theta estimate 

 Eigen::Vector3f g;
 
 g << px -v/w*sin(th) + v/w*sin(th+w*Ts),
      py +v/w*cos(th) - v/w*cos(th+w*Ts),
      th +w*Ts;

  return g;
}

//---------------------------------------------------------------------------

Eigen::MatrixXf FastSlam::getM(const Eigen::VectorXf& u) {

  float v = u(0);    // Velocity input
  float w = u(1);    // Angular velocity input
  Eigen::Matrix2f M; // Noise covariance

    M << alpha1_*powf(v,2)+alpha2_*powf(w,2),              0,
                   0,                            alpha3_*powf(v,2)+alpha4_*powf(w,2); 

    return M;
}


//---------------------------------------------------------------------------

Eigen::Matrix<float,2,2> FastSlam::getH(const Eigen::VectorXf& x, const Eigen::Vector2f& m) {

  float mx = m(0);  // The x position of the landmark seen
  float my = m(1);  // The y position of the landmark seen

  float px = x(0); // X position estimate
  float py = x(1); // Y position estimate
  float th = x(2); // Theta estimate

  float delta_x = mx - px;
  float delta_y = my - py;
  float q = powf(mx-px,2)+powf(my-py,2);
  float qs = sqrt(q);

  Eigen::Matrix<float,2,2> H;

  H << qs*delta_x, qs*delta_y,
            -delta_y, delta_x;
  H = H/q;

  return H;
}

//---------------------------------------------------------------------------

void FastSlam::PropogateTrue(const Eigen::VectorXf& u, float Ts) {

  Eigen::Vector3f x = getg(x_,u,Ts);
  x_.head(3) = x;

  // while (x_(2) > kPI)
  //   x_(2) = x_(2)-2*kPI;
  // while(x_(2)<-kPI)
  //   x_(2) = x_(2) + 2*kPI;

}

//---------------------------------------------------------------------------

void FastSlam::Predict(const Eigen::VectorXf& u, float Ts) {

  float v = u(0);       
  float w = u(1);

  Eigen::Vector2f u_perturbed;

  for (unsigned int k=0; k < num_particles_; k++) {

    // Perturb the state
    v += sqrt((alpha1_*powf(v,2)+alpha2_*powf(w,2)))*randn_(gen_);
    w += sqrt((alpha3_*powf(v,2)+alpha4_*powf(w,2)))*randn_(gen_);
    u_perturbed << v, w;

    Eigen::Vector3f x = getg(particles_[k].stacked_states_,u_perturbed,Ts);
   

    particles_[k].stacked_states_.head(3) = x;

  }

}

//---------------------------------------------------------------------------

bool FastSlam::NewLandmark(const Eigen::VectorXf& z, State& particle, int landmark_id) {

  bool is_new_landmark = false;

  if (!particle.landmarks_seen_[landmark_id])
  {
    Eigen::Vector2f m; // landmark location 

    particle.landmarks_seen_[landmark_id] = true;
    float r = z(0);    // range to target
    float phi = z(1);  // relative heading to target
    float px = particle.stacked_states_(0); // X position estimate
    float py = particle.stacked_states_(1); // Y position estimate
    float th = particle.stacked_states_(2); // Theta estimate

    float mx = px + r*cos(phi+th);
    float my = py+r*sin(phi+th);
    m << mx, my;

    std::cerr << "new meas: " << m <<  std::endl;
    std::cerr << "to landmark: " <<  landmark_id << std::endl;


    particle.landmarks_[landmark_id] = m;

    std::cerr << "added meas" << std::endl;


    Eigen::Matrix<float,2,2> H;

    H = getH(particle.stacked_states_,m);

    // std::cerr << "got H" << std::endl;


    particle.P_[landmark_id] = H.inverse()*Q_*H.inverse().transpose();

    // std::cerr << "cov updated" << std::endl;


    particle.AddWeight(default_weight_);

    // std::cerr << "weight added" << std::endl;



    is_new_landmark = true;

  }

  return is_new_landmark;

}

//---------------------------------------------------------------------------

void FastSlam::Update(const Eigen::VectorXf& z, int landmark_id) {



for (unsigned int k = 0; k < num_particles_; k++) {

  // std::cerr << "update k: " << k << std::endl;

  // If you have never seen the landmark, initilize it
  // else do the normal update
  if (!NewLandmark(z,particles_[k],landmark_id) ) {

    // std::cerr << "not new landmark " << std::endl;

    // Get estimated vector and construct big H
    Eigen::Vector2f zh= getMeasurement(particles_[k].stacked_states_,particles_[k].landmarks_[landmark_id],false,false);
    Eigen::Vector2f err;
    // std::cerr << "got est measurement " << std::endl;

    Eigen::Matrix<float,2,2> K; // Kalman Gain
    Eigen::Matrix<float,2,2> H = getH(particles_[k].stacked_states_,particles_[k].landmarks_[landmark_id]);
    Eigen::Matrix<float,2,2> S =   H*particles_[k].P_[landmark_id]*H.transpose() + Q_;



    K = particles_[k].P_[landmark_id]*H.transpose()*S.inverse();
    // std::cerr << "K: " << std::endl << K_ << std::endl;
    err = (z-zh);

    while (err(1) > kPI)
      err(1) = err(1)-2*kPI;
    while(err(1)<-kPI)
      err(1) = err(1) + 2*kPI;

    // std::cerr << " err: " << std::endl << err << std::endl;



    particles_[k].landmarks_[landmark_id] += K*err;
    particles_[k].P_[landmark_id] = (Eigen::MatrixXf::Identity(2,2)-K*H)*particles_[k].P_[landmark_id];
 
    float weight = ImportanceFactor(err,S);
    particles_[k].AddWeight(weight);

  }
}


}

//---------------------------------------------------------------------------

float FastSlam::ImportanceFactor(const Eigen::Vector2f& err, const Eigen::Matrix<float,2,2> S) {

  float inside = -0.5*err.transpose()*S.inverse()*err;
  float outside = sqrt(2*kPI*S.determinant());
//        std::cerr << "inside: " << std::endl << inside << std::endl;
//      std::cerr << "ExpConst_: " << std::endl << ExpConst_ << std::endl;

  return std::max<float>(0.00001,outside*exp(inside));


}

//---------------------------------------------------------------------------------------------

void FastSlam::Resample() {

  // CalculateTotalWeight();
  NormalizeWeights();

  // std::cerr << "normalized weights" << std::endl;


  particles_tmp_.clear();

  float r = uniform_zero_one_(gen_)/static_cast<float>(num_particles_);

  particles_[0].weight_;
  float c = particles_[0].weight_;


  unsigned int i =0;
  float U = 0;
  float w_t = 0;
  // std::cerr << "starting resample loop" << std::endl;
  
  // std::cerr << "c: " << c << std::endl;

  for (unsigned int k = 0; k < num_particles_; k++) {
    // std::cerr << "here" << std::endl;
      U = r + static_cast<float>(k)/num_particles_;
    // std::cerr << "U: " << U << std::endl;

      while (U>c) {
    // std::cerr << "here3" << std::endl;

          i++;
        // std::cerr << "i: " << i << std::endl;

        c += particles_[i].weight_;
        // std::cerr << "c: " << c << std::endl;
      }
        // std::cerr << "i: " << i << std::endl;

      particles_[i].weight_flag_ = false;
        // std::cerr << "flag set: " << i << std::endl;

      particles_tmp_.push_back(particles_[i]);

        // std::cerr << "pushed back: " << i << std::endl;


  }

  // std::cerr << "finished resample loop" << std::endl;


  particles_ = particles_tmp_;
  NormalizeWeights();
  CalculateParticleMean();


}

//---------------------------------------------------------------------------------------------


void FastSlam::NormalizeWeights() {

  total_weight_ = 0;

  for (unsigned int k = 0; k < num_particles_; k++) {
    total_weight_ += particles_[k].weight_;
  }

  for (unsigned int k = 0; k < num_particles_; k++) {
    particles_[k].weight_ /=total_weight_;
  }


}

//---------------------------------------------------------------------------------------------

void FastSlam::Sim() {

// std::cerr << "start sim" << std::endl;


LogTrueState();
LogParticles();
LogMeanState();
LogErrorCovariance();
LogTime();

// std::cerr << "logged initial data" << std::endl;

Eigen::Vector2f u_true(vel_,angular_vel_);
Eigen::Vector2f u_noise;
Eigen::Matrix2f process_cov = getM(u_true);
process_cov(0,0) = sqrt(process_cov(0,0));
process_cov(1,1) = sqrt(process_cov(1,1));
Eigen::Vector2f z;                           // Measurement
Eigen::Vector2f landmark;                           // Measurement


// std::cerr << "start loop" << std::endl;

while (t_ < tf_) {
  std::cerr << "t: " << t_ << std::endl;
  t_+=Ts_;
  PropogateTrue(u_true,Ts_);
  u_noise = u_true + process_cov*Eigen::Vector2f(randn_(gen_),randn_(gen_));

  // std::cerr << "prop true" << std::endl;

  Predict(u_noise,Ts_);

  // std::cerr << "predict done" << std::endl;

  // std::cerr << "prop true" << std::endl;

  for (unsigned int jj=0; jj < num_landmarks_; jj++) {
    // std::cerr << "landmark: " << jj << std::endl;
    landmark = getLandmark(x_,jj);
    z = getMeasurement(x_, landmark, true,true);

    // TODO: add noise

    LogMeasurement(z);
    // std::cerr << "got meas: " << std::endl << z << std::endl;
    if (!std::isnan(z(0)))
      Update(z,jj);
    else
      std::cerr << "meas not seen" << std::endl;
    // std::cerr << "finished updating" << std::endl;

    // std::cerr << "finished meas update: " << std::endl << std::endl;


  }
    // std::cerr << "finished meas update: " << std::endl << std::endl;


  Resample();
  
  
  LogTrueState();
  LogParticles();
  LogMeanState();
  LogErrorCovariance();
  LogTime();
}

std::cout << "x: " << std::endl << x_ << std::endl;
std::cout << "xh: " << std::endl << xh_ << std::endl;

}