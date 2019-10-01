#include "UKF.h"



UKF::UKF(bool propogateTrue) {

  propogateTrue_ = propogateTrue;
  genSigPoints_ = true;
}

//---------------------------------------------------------------------------

UKF::~UKF() {

}

//---------------------------------------------------------------------------

void UKF::SetMeasurementCovariance(const Eigen::MatrixXf& R){R_ = R;}
void UKF::SetProcessCovariance(const Eigen::MatrixXf& Q){Q_= Q;}
void UKF::SetErrorCovariance(const Eigen::MatrixXf& P){P_ = P;}
void UKF::SetKalmanGain(const Eigen::MatrixXf& K){K_= K;}
void UKF::SetInput(const Eigen::VectorXf& u){u_= u;}
void UKF::SetTrueState(const Eigen::VectorXf& x){x_= x;}
void UKF::SetEstimateState(const Eigen::VectorXf& xh){xh_= xh;}
void UKF::SetTrueMeasurement(const Eigen::VectorXf& z){z_= z;}
void UKF::SetEstimateMeasurement(const Eigen::VectorXf& zh){zh_= zh;}
void UKF::SetWeightParameters(float alpha, float kappa, float beta) { alpha_ = alpha; kappa_ = kappa; beta_ = beta;}

// Set Function Pointers
void UKF::SetSystemFunctionPointer(SysFunPtr f){f_=f;}
void UKF::SetMeasurementFunctionPointer(MeasFunPtr g){g_=g;}
//---------------------------------------------------------------------------

const Eigen::MatrixXf* UKF::GetMeasurementCovariance(){return &R_;}
const Eigen::MatrixXf* UKF::GetProcessCovariance(){return &Q_;}
const Eigen::MatrixXf* UKF::GetErrorCovariance(){return &P_;}
const Eigen::MatrixXf* UKF::GetKalmanGain(){return &K_;}
const Eigen::VectorXf* UKF::GetInput(){return &u_;}
const Eigen::VectorXf* UKF::GetTrueState(){return &x_;}
const Eigen::VectorXf* UKF::GetEstimateState(){return &xh_;}
const Eigen::VectorXf* UKF::GetTrueMeasurement(){return &z_;}
const Eigen::VectorXf* UKF::GetEstimateMeasurement(){return &zh_;}
//---------------------------------------------------------------------------

void UKF::GenerateSigmaPoints() {

  chi_.clear();

  // Construct augmented covariance matrix
  unsigned int dim = P_.rows() + Q_.rows() + R_.rows();
  Eigen::MatrixXf P_aug = Eigen::MatrixXf::Zero(dim,dim);
  P_aug.topLeftCorner(P_.rows(), P_.cols()) = P_;
  P_aug.block(P_.rows(),P_.cols(),Q_.rows(),Q_.cols())=Q_;
  P_aug.bottomRightCorner(R_.rows(),R_.cols())=R_;

  // Construct augmented state vector
  Eigen::VectorXf xh_aug(dim);
  xh_aug.setZero();
  xh_aug.head(xh_.rows())=xh_;
// 
  // std::cout << "P aug: " << std::endl << P_aug << std::endl;
  // std::cout << "xh aug: " << std::endl << xh_aug << std::endl;



  // Cholesky factorization to get lower triangle
  Eigen::LLT<Eigen::MatrixXf> LL(P_aug);
  Eigen::MatrixXf L = LL.matrixL();


  // std::cout << "L: " << std::endl << L << std::endl;
  // std::cout << "LLT: " << std::endl << L*L.transpose() << std::endl;
  // std::cout << "L: " << std::endl << L.col(1) << std::endl;

  // Construct weights
  int N = P_aug.cols();
  float lambda = powf(alpha_,2)*(N+kappa_)-N;
  weight_c_ = lambda/(N+lambda)+(1-pow(alpha_,2)+beta_);
  weight_m_ = lambda/(N+lambda);
  weight_mc_= 0.5/(N+lambda);
  float lambda_n = sqrt(N+lambda);
  // std::cout << "lambda : " << lambda << std::endl;
  // std::cout << "lambda n: " << lambda_n << std::endl;
  // std::cout << "weight_m_ : " << weight_m_ << std::endl;
  // std::cout << "weight_c_ : " << weight_c_ << std::endl;
  // std::cout << "weight_mc_ : " << weight_mc_ << std::endl;


  // Generate Sig Points
  chi_.push_back(xh_aug);
  for (unsigned int i=0; i < N; i++)
  {
    chi_.push_back(xh_aug+lambda_n*L.col(i));
    chi_.push_back(xh_aug-lambda_n*L.col(i));
    // std::cout << "chi: " << i << std::endl << 0*xh_aug+lambda_n*L.col(i) << std::endl << std::endl;
  }



  genSigPoints_ = false;

}

//---------------------------------------------------------------------------

void UKF::Predict(float Ts) {

  if (genSigPoints_)
    GenerateSigmaPoints();

  // Propogate True dynamics if flag is set to true
  if (propogateTrue_) 
    x_ = f_(x_,u_,Ts,true);


  chi_b_.clear();
  for (unsigned int j=0; j < chi_.size(); j++)
  {
    chi_b_.push_back(f_(chi_[j].head(xh_.rows()),u_+chi_[j].segment(xh_.rows(),u_.rows()),Ts,false));
    // std::cout << "chi b: " << j << std::endl << chi_b_[j] << std::endl << std::endl;
    // std::cout << "chi h: " << j << std::endl << chi_[j].head(xh_.rows()) << std::endl << std::endl;

  }


  Eigen::VectorXf mu = weight_m_*chi_b_[0];

  // Compute new mean
  for (unsigned int i = 1; i < chi_b_.size(); i++)
  {
    mu += weight_mc_*chi_b_[i];
  }

  xh_ = mu;

  // Compute new error covariance
  Eigen::MatrixXf P = weight_c_*(chi_b_[0]-mu)*((chi_b_[0]-mu).transpose());
  chi_[0].head(xh_.rows()) = chi_b_[0];
  // std::cout << "P: predict " << std::endl << P << std::endl;

  for (unsigned int j = 1; j < chi_b_.size(); j++)
  {
    P += weight_mc_*(chi_b_[j]-mu)*((chi_b_[j]-mu).transpose());
    chi_[j].head(xh_.rows()) = chi_b_[j];
  // std::cout << "P: predict " << std::endl << P << std::endl;
  
  }

  P_ = P;
 
}

//---------------------------------------------------------------------------

void UKF::Update() {

  if (genSigPoints_)
    GenerateSigmaPoints();

  if (propogateTrue_)
    z_ = g_(x_,u_,true);

  // std::cout << "z: " << std::endl << z_ << std::endl;

  chi_z_.clear();

  // Propagate sigma points though measurment funciton
  for (unsigned int i =0; i < chi_b_.size(); i++)
    chi_z_.push_back(g_(chi_[i].head(xh_.rows()),u_,false)+chi_[i].tail(R_.rows()));

  // Compute mean and standar deviation of the measurment

  Eigen::VectorXf Z = weight_m_*chi_z_[0];

  // Compute new mean
  for (unsigned int i = 1; i < chi_z_.size(); i++)
  {
    Z += weight_mc_*chi_z_[i];
  }

  zh_ = Z;

  // Compute new error covariance
  Eigen::MatrixXf S = weight_c_*(chi_z_[0]-Z)*(chi_z_[0]-Z).transpose();

  for (unsigned int j = 1; j < chi_z_.size(); j++)
  {
    S += weight_mc_*(chi_z_[j]-Z)*(chi_z_[j]-Z).transpose();
  }

  // Compute Cross correlation
  Eigen::MatrixXf Pc = weight_c_*(chi_[0].head(xh_.rows())-xh_)*(chi_z_[0]-Z).transpose();

  for (unsigned int j = 1; j < chi_z_.size(); j++)
  {
    Pc += weight_mc_*(chi_[j].head(xh_.rows())-xh_)*(chi_z_[j]-Z).transpose();
  }

  K_ = Pc*S.inverse();
  xh_ = xh_ + K_*(z_-zh_);
  P_ = P_ - K_*S*K_.transpose();

  // std::cout << "zh: " << std::endl << zh_ << std::endl;
  // std::cout << "S: " << std::endl << S << std::endl;
  // S.inverse();
  // std::cout << "Si: " << std::endl << S << std::endl;
  // std::cout << "x: " << std::endl << x_ << std::endl;
  // std::cout << "xh: " << std::endl << xh_ << std::endl;
  // std::cout << "P: " << std::endl << P_ << std::endl;

  // After every update, new sig points will need to be generated
  genSigPoints_ = true;

}