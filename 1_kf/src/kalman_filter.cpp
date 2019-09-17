#include "kalman_filter.h"


namespace kf {

KalmanFilter::KalmanFilter(bool continuousSystem) {

  cont_sys_ = continuousSystem;
  need_discretize_ = false;

}



//-------------------------------------------------------------------------------------------------------------------------

KalmanFilter::~KalmanFilter()
{

}

// Set the initial estimate of xh

void KalmanFilter::SetInitialEstimateState(const Eigen::VectorXf& x)
{
  x_ = x;
}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetObservationMatrix(const Eigen::MatrixXf& C)
{
  C_ = C;
}

//-------------------------------------------------------------------------------------------------------------------------


void KalmanFilter::SetSystemMatrixCont(const Eigen::MatrixXf& Ac)
{
  Ac_ = Ac;
  need_discretize_= true;
}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetSystemMatrixDiscrete(const Eigen::MatrixXf& Ad)
{
  Ad_ = Ad;
}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetInputMatrixCont(const Eigen::MatrixXf& Bc)
{
  Bc_ = Bc;
  need_discretize_ = true;
}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetInputMatrixDiscrete(const Eigen::MatrixXf& Bd)
{
  Bd_ = Bd;
}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetNoiseMatrixCont(const Eigen::MatrixXf& Gc)
{
  Gc_ = Gc;
  need_discretize_ = true;
}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetNoiseMatrixDiscrete(const Eigen::MatrixXf& Gd)
{
  Gd_ = Gd;
}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetR(const Eigen::MatrixXf& R)
{
  R_ = R;
}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetQCont(const Eigen::MatrixXf& Qc)
{
  Qc_ = Qc;
  need_discretize_ = true;
}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetQDiscrete(const Eigen::MatrixXf& Qd)
{
  Qd_ = Qd;
}


//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::SetCovarianceMatrix(const Eigen::MatrixXf& P)
{
  P_ = P;
}

void KalmanFilter::Discretize(float Ts)
{

  Eigen::MatrixXf Id = Eigen::MatrixXf::Identity(Ac_.cols(),Ac_.cols());
  Eigen::MatrixXf tmp = Ac_*Ts;
  Eigen::MatrixXf tmp2 = (Id +tmp/2+tmp*tmp/6+ tmp*tmp*tmp/24 + tmp*tmp*tmp*tmp/120);

  Ad_ = Id + tmp + tmp*tmp/2 + tmp*tmp*tmp/6 + tmp*tmp*tmp*tmp/24 + tmp*tmp*tmp*tmp*tmp/120;
  Bd_=tmp2*Bc_*Ts;
  Gd_ = tmp2*Gc_*Ts;
  Qd_ = Gd_*Qc_*Gd_.transpose();

  Ts_ = Ts;

  need_discretize_ = false;

}


//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::Predict(float Ts, const Eigen::VectorXf& u)
{
  if (need_discretize_ || Ts != Ts_)
  {
    Discretize(Ts);
  }

  x_ = Ad_*x_ + Bd_*u;
  P_ = Ad_*P_*Ad_.transpose() + Qd_;

}

//-------------------------------------------------------------------------------------------------------------------------

void KalmanFilter::Update(const Eigen::VectorXf& y)
{
  Eigen::MatrixXf K;
  Eigen::MatrixXf temp;
  temp = C_*P_*C_.transpose()+R_;
  K = P_*C_.transpose()*(temp.inverse());
  x_ = x_+K*(y-C_*x_);
  P_=(Eigen::MatrixXf::Identity(Ac_.cols(),Ac_.cols()) -K*C_)*P_;
}



// Return a pointer to the estimated state
const Eigen::VectorXf* KalmanFilter::GetEstimatedState()
{
  return &x_;
}


// Return a pointer to the discrete input matrix
const Eigen::MatrixXf* KalmanFilter::GetInputMatrixDiscrete()
{
  return &Bd_;
}

// Return a pointer to the discrete input matrix
const Eigen::MatrixXf* KalmanFilter::GetSystemMatrixDiscrete()
{
  return &Ad_;
}

// Return a pointer to the discrete input matrix
const Eigen::MatrixXf* KalmanFilter::GetNoiseMatrixDiscrete()
{
  return &Gd_;
}

// Return a pointer to the discrete process noise
const Eigen::MatrixXf* KalmanFilter::GetProcessNoiseCovDiscrete()
{
  return &Qd_;
}





}