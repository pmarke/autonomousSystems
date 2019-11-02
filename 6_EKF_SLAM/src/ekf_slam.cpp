#include "ekfslam/ekf_slam.h"


EKF_SLAM::EKF_SLAM(int num_landmarks, float vel, float radius)
{


// Initilize logging data
std::cout << "Writting log data to /home/mark/projects/autonomousSystems/6_EKF_SLAM/log" << std::endl;
  
// Make sure the filepath exists
if (!fs::exists("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log"))
{
  fs::create_directory("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log");
}

// log_K_.open("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/k.log",std::ofstream::out);
log_t_.open("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/t.log",std::ofstream::out);
log_x_.open("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/x.log",std::ofstream::out);
log_xh_.open("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/xh.log",std::ofstream::out); 
log_u_.open("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/u.log",std::ofstream::out); 
log_P_.open("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/P.log",std::ofstream::out); 
log_m_.open("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/m.log",std::ofstream::out);
log_landmarks_.open("/home/mark/projects/autonomousSystems/6_EKF_SLAM/log/landmarks.log",std::ofstream::out);
log_landmarks_.write( (char*) &num_landmarks,   sizeof(int));


vel_ = vel;
radius_ = radius;
angular_vel_ = vel_/radius_;
if (num_landmarks < 1)
  num_landmarks = 1;
num_landmarks_ = num_landmarks;




// Init meas covariance
Q_ << powf(sigma_r_,2), 0, 0, powf(sigma_phi_,2);
const int dim = 3+2*num_landmarks_;
P_ = Eigen::MatrixXf::Zero(dim,dim);
P_.setIdentity();
P_ = P_*100;
P_.block<3,3>(0,0)=Eigen::Matrix<float,3,3>::Zero();

// std::cout << "P matrix: " << std::endl << P_ << std::endl;

InitEnvironment();


}

//---------------------------------------------------------------------------

EKF_SLAM::~EKF_SLAM()
{

  // log_K_.close();                 
  log_t_.close();                
  log_x_.close();  
  log_xh_.close();  
  log_u_.close();  
  log_P_.close(); 
  log_m_.close();
  log_landmarks_.close();
  std::cout << "Finished writing log data!" << std::endl;

}


//---------------------------------------------------------------------------


void EKF_SLAM::InitEnvironment()
{
  
t_ = 0.0f;
Ts_ = 0.1f;
tf_ = 2.0f*kPI*radius_/vel_*2.0f;  // approximate time for one orbit

// Initialize the true and estimated states
x_ = Eigen::MatrixXf::Zero(3+2*num_landmarks_,1);
x_(0) = 0.0f;
x_(1) = 0.0f;
x_(2) = kPI/2.0f;
xh_ = Eigen::MatrixXf::Zero(3+2*num_landmarks_,1);
xh_.head(3) = x_.head(3);
// xh_.head(3) << 5.0*randn_(gen_), 5.0f*randn_(gen_), kPI*randn_(gen_);
// if (xh_(2) > kPI)
//   xh_(2) = xh_(2)-2*kPI;
// if(xh_(3)<-kPI)
//   xh_(2) = xh_(2) + 2*kPI;

// Initilize the landmark vector
landmark_seen_ = std::vector<bool>(num_landmarks_,false);

// Create Landmarks
Eigen::Vector2f m;
Eigen::Vector2f z;
z << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN();
m << x_(0), x_(1)+radius_;
setLandmark(x_,1,m);
LogMeasurement(z);
log_landmarks_.write( (char*) m.data(),   sizeof(float)*2);

float w_sign = 1;
if (angular_vel_ > 0)
  w_sign = -1;

if (num_landmarks_ > 1) {
  float delta_th = 2.0f*kPI/(num_landmarks_);
  float curr_th = delta_th;
  Eigen::Vector2f center(x_(0)+radius_*w_sign,x_(1));
  for (unsigned int ii = 2; ii <= num_landmarks_; ii++) {
    m << center(0)-radius_*cos(curr_th)*w_sign, center(1) + radius_*sin(curr_th);
    curr_th += delta_th;
    setLandmark(x_,ii,m);
    LogMeasurement(z);
    log_landmarks_.write( (char*) m.data(),   sizeof(float)*2);
  }
}



}

//---------------------------------------------------------------------------

Eigen::Vector2f EKF_SLAM::getLandmark(const Eigen::VectorXf& x, int landmark_id) {

  Eigen::Vector2f landmark; // Measurement 

  float mx = x(1+2*landmark_id);  // The x position of the landmark seen
  float my = x(2+2*landmark_id);  // The y position of the landmark seen

  landmark << mx, my;

  return landmark;
}

//---------------------------------------------------------------------------

void EKF_SLAM::setLandmark(Eigen::VectorXf& x, int landmark_id, const Eigen::Vector2f& m) {
  x(1+2*landmark_id) = m(0);
  x(2+2*landmark_id) = m(1);
}

//---------------------------------------------------------------------------


Eigen::VectorXf EKF_SLAM::getg(const Eigen::VectorXf& x, const Eigen::VectorXf& u, float Ts) {

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

Eigen::MatrixXf EKF_SLAM::getG(const Eigen::VectorXf& x, const Eigen::VectorXf& u, float Ts) {

  float v = u(0);   // Velocity input
  float w = u(1);   // Angular velocity input

  float px = x(0); // X position estimate
  float py = x(1); // Y position estimate
  float th = x(2); // Theta estimate 

  Eigen::Matrix3f G;

  G << 0,    0,  -v/w*cos(th)+v/w*cos(th+w*Ts),
       0,    0,  -v/w*sin(th)+v/w*sin(th+w*Ts),
       0,    0,               0; 

  return G;

}

//---------------------------------------------------------------------------

Eigen::MatrixXf EKF_SLAM::getV(const Eigen::VectorXf& x, const Eigen::VectorXf& u, float Ts) {


  float v = u(0);   // Velocity input
  float w = u(1);   // Angular velocity input

  float px = x(0); // X position estimate
  float py = x(1); // Y position estimate
  float th = x(2); // Theta estimate 

  Eigen::Matrix<float,3,2> V;

    V << (-sin(th)+sin(th+w*Ts))/w, v*(sin(th)-sin(th+w*Ts))/powf(w,2)+ v*cos(th+w*Ts)*Ts/w,
          (cos(th)-cos(th+w*Ts))/w, -v*(cos(th)-cos(th+w*Ts))/powf(w,2)+ v*sin(th+w*Ts)*Ts/w,
            0, Ts;

  return V;

}

//---------------------------------------------------------------------------

Eigen::MatrixXf EKF_SLAM::getM(const Eigen::VectorXf& u) {

  float v = u(0);    // Velocity input
  float w = u(1);    // Angular velocity input
  Eigen::Matrix2f M; // Noise covariance

    M << alpha1_*powf(v,2)+alpha2_*powf(w,2),              0,
                   0,                            alpha3_*powf(v,2)+alpha4_*powf(w,2); 

    return M;
}

//---------------------------------------------------------------------------

void EKF_SLAM::PropogateTrue(const Eigen::VectorXf& u, float Ts) {

  Eigen::Vector3f x = getg(x_,u,Ts);
  x_.head(3) = x;

  // while (x_(2) > kPI)
  //   x_(2) = x_(2)-2*kPI;
  // while(x_(2)<-kPI)
  //   x_(2) = x_(2) + 2*kPI;

}

//---------------------------------------------------------------------------

void EKF_SLAM::Predict(const Eigen::VectorXf& u, float Ts) {

  Eigen::Vector3f x = getg(xh_,u,Ts);
  Eigen::Matrix3f g =getG(xh_,u,Ts);
  Eigen::Matrix<float,3,2> V = getV(xh_,u,Ts);
  Eigen::Matrix2f M = getM(u);
  Eigen::MatrixXf F = Eigen::MatrixXf::Zero(3,3+2*num_landmarks_);
  Eigen::MatrixXf G = Eigen::MatrixXf::Zero(3+2*num_landmarks_,3+2*num_landmarks_);
  F.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
  G = Eigen::MatrixXf::Identity(3+2*num_landmarks_,3+2*num_landmarks_)+F.transpose()*g*F;


  // std::cerr << "V: " << std::endl << V << std::endl;
  // std::cerr << "G: " << std::endl << G << std::endl;
  // std::cerr << "F: " << std::endl << F << std::endl;
  // std::cerr << "M: " << std::endl << M << std::endl;

  xh_.head(3) = x;
  // while (xh_(2) > kPI)
  //   xh_(2) = xh_(2)-2*kPI;
  // while(xh_(2)<-kPI)
  //   xh_(2) = xh_(2) + 2*kPI;
  // P_.block<3,3>(0,0) = G*P_.block<3,3>(0,0)*G.transpose() + V*M*V.transpose();
  P_ = G*P_*G.transpose() + F.transpose()* V*M*V.transpose()*F;

  // std::cerr << "P_.block<3,3>(0,0): " << std::endl << P_.block<3,3>(0,0) << std::endl;


}

//---------------------------------------------------------------------------

Eigen::Vector2f EKF_SLAM::getMeasurement(const Eigen::VectorXf& x,int landmark_id, bool flag_true, bool flag_noise) {

  Eigen::Vector2f z; // Measurement 
  Eigen::Vector2f m; // landmark location 
  m = getLandmark(x,landmark_id);

  // std::cerr << "m: " << std::endl << m << std::endl;
  // std::cerr << "landmark_id: " << std::endl << landmark_id << std::endl;

  

  float mx = m(0);  // The x position of the landmark seen
  float my = m(1);  // The y position of the landmark seen

  float px = x(0); // X position estimate
  float py = x(1); // Y position estimate
  float th = x(2); // Theta estimate

  float q = powf(mx-px,2)+powf(my-py,2);    // Range^2 to landmark

  if (flag_noise)
    z << sqrt(q) + sigma_r_*randn_(gen_),atan2(my-py,mx-px)+sigma_phi_*randn_(gen_)-th;
  else
    z << sqrt(q),atan2(my-py,mx-px)-th;

  // std::cout << "zb: " << z(1) << std::endl;


  // while (z(1) > kPI)
  //   z(1) = z(1)-2*kPI;
  // while(z(1)<-kPI)
  //   z(1) = z(1) + 2*kPI;

  // std::cout << "z: " << z(1) << std::endl;

  // See if the measurement is in the field of view if
  // it is associated with the true state
  // if it isn't in the field of view. Set it to Nan
  if( fabs(z(1))>fov_*100 && flag_true)
    z << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN();

  return z;
}

//---------------------------------------------------------------------------


Eigen::Matrix<float,2,5> EKF_SLAM::getSmallH(int landmark_id) {

  Eigen::Vector2f m; // landmark location 
  m = getLandmark(xh_,landmark_id);
  

  float mx = m(0);  // The x position of the landmark seen
  float my = m(1);  // The y position of the landmark seen

  float px = xh_(0); // X position estimate
  float py = xh_(1); // Y position estimate
  float th = xh_(2); // Theta estimate

  float delta_x = mx - px;
  float delta_y = my - py;
  float q = powf(mx-px,2)+powf(my-py,2);
  float qs = sqrt(q);

  Eigen::Matrix<float,2,5> H;

  H << -qs*delta_x, -qs*delta_y, 0, qs*delta_x, qs*delta_y,
           delta_y, -delta_x,   -q,   -delta_y, delta_x;
  H = H/q;

  return H;

}

//---------------------------------------------------------------------------

void EKF_SLAM::NewLandmark(const Eigen::VectorXf& z, int landmark_id) {

  if (!landmark_seen_[landmark_id])
  {
    Eigen::Vector2f m; // landmark location 

    landmark_seen_[landmark_id] = true;
    float r = z(0);    // range to target
    float phi = z(1);  // relative heading to target
    float px = xh_(0); // X position estimate
    float py = xh_(1); // Y position estimate
    float th = xh_(2); // Theta estimate

    float mx = px + r*cos(phi+th);
    float my = py+r*sin(phi+th);
    m << mx, my;

    // std::cerr << "new landmark m: " << std::endl << m << std::endl;

    setLandmark(xh_,landmark_id,m);

  }

}

//---------------------------------------------------------------------------

void EKF_SLAM::Update(const Eigen::VectorXf& z, int landmark_id) {

  // If you have never seen the landmark, initilize it
  // else do the normal update
  NewLandmark(z,landmark_id);

  // std::cerr << "new landmark " << std::endl;

  // Get estimated vector and construct big H
  Eigen::Vector2f zh= getMeasurement(xh_,landmark_id,false,false);
  Eigen::Vector2f err;
  // std::cerr << "got est measurement " << std::endl;

  Eigen::Matrix<float,2,5> h = getSmallH(landmark_id);
  // std::cerr << "got small h " << std::endl;

  Eigen::MatrixXf H = Eigen::MatrixXf::Zero(2,xh_.rows());
  H.setZero();

  H.block<2,3>(0,0) = h.block<2,3>(0,0);
  H.block<2,2>(0,1+2*landmark_id) = h.block<2,2>(0,3);

  // std::cerr << "constructed big h " << std::endl;
  // std::cerr << "xh: " << std::endl << xh_.head(3) << std::endl;
  // std::cerr << "zh: " << std::endl << zh << std::endl;
  // std::cerr << "z: " << std::endl << z << std::endl;
  // std::cerr << "h: " << std::endl << h << std::endl;
  // std::cerr << "landmark_id: " << std::endl << landmark_id << std::endl;

  // std::cerr << "H: " << std::endl << H << std::endl;
  // std::cerr << "zh: " << std::endl << zh << std::endl;


  S_ = H*P_*H.transpose() + Q_;
  // std::cerr << "S: " << std::endl << S_ << std::endl;

  K_ = P_*H.transpose()*S_.inverse();
  // std::cerr << "K: " << std::endl << K_ << std::endl;
  err = (z-zh);

  while (err(1) > kPI)
    err(1) = err(1)-2*kPI;
  while(err(1)<-kPI)
    err(1) = err(1) + 2*kPI;

  std::cerr << " err: " << std::endl << err << std::endl;



  xh_ += K_*err;
  // std::cerr << "xh_: " << std::endl << xh_ << std::endl;
  P_ = (Eigen::MatrixXf::Identity(3+2*num_landmarks_,3+2*num_landmarks_)-K_*H)*P_;
  // std::cerr << "P_: " << std::endl << P_ << std::endl;

  // while (xh_(2) > kPI)
  //   xh_(2) = xh_(2)-2*kPI;
  // while(xh_(2)<-kPI)
  //   xh_(2) = xh_(2) + 2*kPI;

}

//---------------------------------------------------------------------------

void EKF_SLAM::LogTrueState() {
  log_x_.write( (char*) x_.data(),  3*sizeof(float));
}

//---------------------------------------------------------------------------

void EKF_SLAM::LogEstState() {
  log_xh_.write( (char*) xh_.data(), (3+num_landmarks_*2)*sizeof(float));
}
//---------------------------------------------------------------------------

void EKF_SLAM::LogErrorCovariance() {

  // std::cerr << "Pd " << std::endl << P_.diagonal() << std::endl;
  Eigen::VectorXf Pd= P_.diagonal();
  log_P_.write( (char*) Pd.data(),   (3+2*num_landmarks_)*sizeof(float)); 
}

//---------------------------------------------------------------------------

void EKF_SLAM::LogMeasurement(const Eigen::Vector2f& z)
{
  log_m_.write( (char*) z.data(),   2*sizeof(float));
}

//---------------------------------------------------------------------------

void EKF_SLAM::LogData()
{
    // log data
    // log_K_.write( (char*) K_.data(),   6*sizeof(float));  
    log_u_.write( (char*) u_.data(),   2*sizeof(float));  
    // std::cerr << "one " << std::endl;             
    log_t_.write( (char*) &t_,          sizeof(float));                 
    LogTrueState();
    LogEstState();
    LogErrorCovariance();
}

//---------------------------------------------------------------------------

void EKF_SLAM::Sim() {

// std::cerr << "start sim" << std::endl;


u_ << 0.0f,0.0f;
LogData();

// std::cerr << "logged initial data" << std::endl;

Eigen::Vector2f u_true(vel_,angular_vel_);
Eigen::Matrix2f process_cov = getM(u_true);
Eigen::Vector2f z;                           // Measurement


// std::cerr << "start loop" << std::endl;

while (t_ < tf_) {
  // std::cerr << "t: " << t_ << std::endl;
  t_+=Ts_;
  PropogateTrue(u_true,Ts_);
  u_ = u_true + process_cov*Eigen::Vector2f(randn_(gen_),randn_(gen_));

  // std::cerr << "prop true" << std::endl;

  Predict(u_,Ts_);

  // std::cerr << "predict done" << std::endl;

  // std::cerr << "prop true" << std::endl;

  for (unsigned int jj=1; jj <= num_landmarks_; jj++) {
    // std::cerr << "landmark: " << jj << std::endl;
    z = getMeasurement(x_, jj, true,false);

    // TODO: add noise

    LogMeasurement(z);
    // std::cerr << "got meas: " << std::endl << z << std::endl;
    if (!std::isnan(z(0)))
      Update(z,jj);
    else
      std::cerr << "meas not seen" << std::endl;
    // std::cerr << "finished updating" << std::endl;

  }
  

  LogData();
}

}
