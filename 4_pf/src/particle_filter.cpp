#include "particle_filter.h"


ParticleFilter::ParticleFilter(bool propogateTrue, unsigned int M) {
    propogateTrue_ = propogateTrue;
    M_ = M;
    chi_ = new Eigen::VectorXf[M];
    chib_ = new Eigen::VectorXf[M];
    zh_ = new Eigen::VectorXf[M];
    R_ << 0.1, 0, 0, 0.05;
    ExpConst_ = sqrt(powf(2*kPI,2)*R_.determinant());
    InitializeEstimate();
}

//---------------------------------------------------------------------------------------------

ParticleFilter::~ParticleFilter() {
    delete [] chi_;
    delete [] chib_;
    delete [] zh_;
    chi_ = nullptr;
    chib_ = nullptr;
    zh_ = nullptr;
}

//---------------------------------------------------------------------------------------------

void ParticleFilter::SetInput(const Eigen::VectorXf& u){u_= u;}
void ParticleFilter::SetTrueState(const Eigen::VectorXf& x){x_= x;}
//void ParticleFilter::SetEstimateState(const Eigen::VectorXf& xh){xh_= xh;}
void ParticleFilter::SetTrueMeasurement(const Eigen::VectorXf& z){z_= z;}
//void ParticleFilter::SetEstimateMeasurement(const Eigen::VectorXf& zh){zh_= zh;}

// Set Function Pointers
void ParticleFilter::SetSystemFunctionPointer(SysFunPtr f){f_=f;}
void ParticleFilter::SetMeasurementFunctionPointer(MeasFunPtr g){g_=g;}

const Eigen::VectorXf* ParticleFilter::GetInput(){return &u_;}
const Eigen::VectorXf* ParticleFilter::GetTrueState(){return &x_;}
const Eigen::Vector3f* ParticleFilter::GetEstimateState(){return &xh_;}
const Eigen::VectorXf* ParticleFilter::GetParticles(){return chi_;}
const Eigen::VectorXf* ParticleFilter::GetTrueMeasurement(){return &z_;}
//const Eigen::VectorXf* ParticleFilter::GetEstimateMeasurement(){return &zh_;}

//---------------------------------------------------------------------------------------------

void ParticleFilter::InitializeEstimate() {

    xh_.setZero();
    for (int i = 0; i  < M_; i++) {
        chi_[i] = Eigen::Vector4f(uniform_x_(gen_),uniform_y_(gen_),uniform_th_(gen_), 1.0/ static_cast<float>(M_));
        xh_ += chi_[i].head(3)*chi_[i](3);
//      std::cerr << "chi: " << std::endl << chi_[i] << std::endl;
    }


//    std::cerr << "done initializing estiamte: " << std::endl;

}

//---------------------------------------------------------------------------------------------

void ParticleFilter::Predict(float Ts) {

    // Propogate True dynamics if flag is set to true
    if (propogateTrue_)
        x_ = f_(x_,u_,Ts,true);

//    std::cerr << "x_: " << std::endl << x_ << std::endl;

    for (unsigned int i=0; i < M_; i++)
    {
        chi_[i] = f_(chi_[i],u_,Ts,true);
//        std::cerr << "chi: " << std::endl << chi_[i] << std::endl;

    }

//  std::cerr << "done predicting: " << std::endl;


}
//---------------------------------------------------------------------------------------------

void ParticleFilter::Update() {

    if (propogateTrue_)
        z_ = g_(x_,u_,true);

    float w_t = 0;
    for (unsigned int i=0; i < M_; i++)
    {
        zh_[i] = g_(chi_[i],u_,false);
        chi_[i](3) = GetWeight(z_,zh_[i]);
//        float weight = GetWeight(z_,zh_[i]);
//        if (weight > 0.1) {
//          std::cerr << "weight: " << std::endl << weight << std::endl;
//          std::cerr << "chi: " << std::endl << chi_[i] << std::endl;
//
//        }


          w_t += chi_[i](3);
    }
    for (unsigned int i=0; i < M_; i++)
    {
        chi_[i](3) /=w_t;
//        float weight = chi_[i](3);
//        if (weight > 0.001) {
//          std::cerr << "weight: " << std::endl << weight << std::endl;
//          std::cerr << "chi: " << i << std::endl << chi_[i] << std::endl;
//
//        }

    }

//  std::cerr << "done update: " << std::endl;

  Resample();

//  std::cerr << "done resample: " << std::endl;


}

//---------------------------------------------------------------------------------------------

void ParticleFilter::Resample() {

    std::uniform_real_distribution<float> uniform_r{0, 1/static_cast<float>(M_)};
    float r = uniform_r(gen_);
//    std::cerr << "r: " << std::endl << r << std::endl;


    float c = chi_[0](3);
//    std::cerr << "c: " << std::endl << c << std::endl;

    unsigned int i =0;
    float U = 0;
    float w_t = 0;
    for (unsigned int j = 0; j < M_; j++) {
        U = r + static_cast<float>(j)/M_;
        while (U>c) {
            i++;
            c += chi_[i](3);
        }
        chib_[j] = chi_[i];
        w_t += chi_[i](3);
//        std::cerr << "chi: " << std::endl << chi_[i] << std::endl;


    }

    xh_.setZero();
    for (unsigned int j = 0; j < M_; j++)
    {
        chi_[j] = chib_[j];
        chi_[j](3)/=w_t;
//        std::cerr << "chi: " << std::endl << chi_[j] << std::endl;

      xh_ += chi_[j].head(3)*chi_[j](3);
    }


}

//---------------------------------------------------------------------------------------------

float ParticleFilter::GetWeight(Eigen::Vector2f z, Eigen::Vector2f zh) {
  float inside = -0.5*(z-zh).transpose()*R_.inverse()*(z-zh);
//        std::cerr << "inside: " << std::endl << inside << std::endl;
//      std::cerr << "ExpConst_: " << std::endl << ExpConst_ << std::endl;

  return ExpConst_*exp(inside);
}

