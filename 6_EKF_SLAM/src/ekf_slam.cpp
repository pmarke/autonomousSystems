#include "ekf_slam.h"


EKF_SLAM::EKF_SLAM(int num_landmarks)
{




}

//---------------------------------------------------------------------------

EKF_SLAM::~EKF_SLAM()
{
}


//---------------------------------------------------------------------------


void EKF_SLAM::InitLandmarks()
{
  


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

void EKF_SLAM::setLandmark(const Eigen::VectorXf& x, int landmark_id, const Eigen::Vector2f& m) {
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

  G << 1,    0,  -v/w*cos(th)+v/w*cos(th+w*Ts),
       0,    1,  -v/w*sin(th)+v/w*sin(th+w*Ts),
       0,    0,               1; 

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

}

//---------------------------------------------------------------------------

void EKF_SLAM::Predict(const Eigen::VectorXf& u, float Ts) {

  Eigen::Vector3f x = getg(xh_,u,Ts);
  Eigen::Matrix3f G =getG(xh_,u,Ts);
  Eigen::Matrix<float,3,2> V = getV(xh_,u,Ts);
  Eigen::Matrix2f M = getM(u,Ts);

  xh_.head(3) = x;
  P.block<0,0,3,3> = G*P.block<0,0,3,3>*G.transpose() + V*M*V.transpose();

}

//---------------------------------------------------------------------------

Eigen::VectorXf EKF_SLAM::getMeasurement(const Eigen::VectorXf& x,int landmark_id,, bool flag_true) {

  Eigen::Vector2f z; // Measurement 
  Eigen::Vector2f m; // landmark location 
  m = getLandmark(x,landmark_id)
  

  float mx = m(0);  // The x position of the landmark seen
  float my = m(1);  // The y position of the landmark seen

  float px = x(0); // X position estimate
  float py = x(1); // Y position estimate
  float th = x(2); // Theta estimate

  float q = powf(mx-px,2)+powf(my-py,2);    // Range^2 to landmark

  z << sqrt(q) + sigma_r_*randn_(gen_),atan2(my-py,mx-px)+sigma_phi_*randn_(gen_)-th;

  // See if the measurement is in the field of view if
  // it is associated with the true state
  // if it isn't in the field of view. Set it to Nan
  if( fabs(z(1))<fov_/2.0 && flag_true)
    z << std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN();

  return z;
}

//---------------------------------------------------------------------------


Eigen::Matrix<float,2,5> EKF_SLAM::getSmallH(int landmark_id) {

  Eigen::Vector2f m; // landmark location 
  m = getLandmark(xh_,landmark_id)
  

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
       delta_y, -delta_x, -q, -delta_y, delta_x;
  H = H/q;

  return H;

}

//---------------------------------------------------------------------------

void NewLandmark(const Eigen::VectorXf& z, int landmark_id) {

  if (!landmark_seen_(landmark_id))
  {
    Eigen::Vector2f m; // landmark location 

    landmark_seen_(landmark_id) = true;
    float r = z(0);    // range to target
    float phi = z(1);  // relative heading to target
    float px = xh_(0); // X position estimate
    float py = xh_(1); // Y position estimate
    float th = xh_(2); // Theta estimate

    float mx = px + r*cos(phi+th);
    float my = py+r*sin(phi+th);
    m << mx, my;

    setLandmark(x,landmark_id,m);

  }

}

//---------------------------------------------------------------------------

void EKF_SLAM::Update(const Eigen::VectorXf& z, int landmark_id) {

  // If you have never seen the landmark, initilize it
  // else do the normal update
  NewLandmark(z,landmark_id);

  // Get estimated vector and construct big H
  Eigen::Vector2f z_est = getMeasurement(xh_,landmark_id,false);
  Eigen::Matrix<float,2,5> h = getSmallH(landmark_id);
  Eigen::Matrix<float,5,xh_.rows()>

}