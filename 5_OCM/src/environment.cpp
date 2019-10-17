#include "environment.h"



ENV::ENV(const char* matlab_file_name) {

  InitMatlabData(matlab_file_name);
  InitMap();

}

//-----------------------------------------------------------------------------

ENV::~ENV() {

}

//-----------------------------------------------------------------------------

void ENV::InitMatlabData(const char* matlab_file_name) {

  // Open mat file
  MATFile *pmat;
  pmat = matOpen(matlab_file_name,"r");
  if (pmat == nullptr)
    std::cerr << "file not found" << std::endl;

  // std::cerr << "here1 " << std::endl;

  CreateMatlabStruct(pmat, "z", z_data_);

  // std::cerr << "here2 " << std::endl;

  CreateMatlabStruct(pmat, "map", map_data_);

  // std::cerr << "here3 " << std::endl;

  CreateMatlabStruct(pmat, "X", X_data_);

  // std::cerr << "here4 " << std::endl;

  CreateMatlabStruct(pmat, "thk", thk_data_);

  // std::cerr << "here5 " << std::endl;


  matClose(pmat);

  // std::cerr << "done " << std::endl;

}

//-----------------------------------------------------------------------------

void ENV::CreateMatlabStruct(MATFile* pmat, const char* var_name, MATLAB_DATA& mat_struct) {

  mxArray *var = matGetVariable(pmat,var_name);

  if (mxIsEmpty(var))
    std::cerr << "variable " << var_name << " not found" << std::endl;  
  else
  {
    // Get dimension of matrix
    size_t var_num_dims = mxGetNumberOfDimensions(var);

    // Get size for each dimension
    const mwSize *var_size =  mxGetDimensions(var);

    // Get total number of elements in array. This is the product of the
    // size of every dimension
    size_t var_num_elements = mxGetNumberOfElements(var);

    mxDouble* var_data = mxGetDoubles(var);

    mat_struct.num_elements = var_num_elements;
    mat_struct.num_dims = var_num_dims;

    double* data_cpy = new double[var_num_elements];
    size_t* size_cpy = new size_t[var_num_dims];

    memcpy(data_cpy,var_data,var_num_elements*8);
    memcpy(size_cpy,var_size,var_num_dims*sizeof(mwSize));

    mat_struct.data = data_cpy;
    data_cpy = nullptr;
    mat_struct.size = size_cpy;
    size_cpy = nullptr;

    mxDestroyArray(var);

  }

}

//-----------------------------------------------------------------------------

void ENV::InitMap() {

  std::cout << "size: " << map_data_.size[0] << std::endl;

  map_true_ = cv::Mat(cv::Size(map_data_.size[0],map_data_.size[1]),CV_32FC3);
  map_est_ = cv::Mat(cv::Size(map_data_.size[0],map_data_.size[1]),CV_32FC1);;
  map_est_.setTo(cv::Scalar::all(0.5));

  cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
  // cv::imshow( "Display window", map_est_ );                   // Show our image inside it.

  cv::waitKey(0); 

}