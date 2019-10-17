#include "environment.h"



ENV::ENV() {

  // InitMatlabData(matlab_file_name);
  InitMap();

}

//-----------------------------------------------------------------------------

ENV::~ENV() {

}

//-----------------------------------------------------------------------------

void ENV::InitMatlabData() {

  std::ifstream thkFile("/home/mark/projects/autonomousSystems/5_OCM/matlab/thk.bin",std::ifstream::in | std::ifstream::binary);
  std::ifstream stateFile("/home/mark/projects/autonomousSystems/5_OCM/matlab/state.bin",std::ifstream::in | std::ifstream::binary);
  std::ifstream measFile("/home/mark/projects/autonomousSystems/5_OCM/matlab/meas.bin",std::ifstream::in | std::ifstream::binary);
  std::ifstream mapFile("/home/mark/projects/autonomousSystems/5_OCM/matlab/map.bin",std::ifstream::in | std::ifstream::binary);


  double current;
  char data[sizeof(current)];


  while (!thkFile.eof()) {
    thkFile.read(data, sizeof(double));
    memcpy(&current, data,sizeof(double));    
    if(thkFile.eof()) break;
    thk_data_.data.push_back(current);
  }



  thkFile.close()
  stateFile.close()
  measFile.close()
  mapFile.close()

}



//-----------------------------------------------------------------------------

void ENV::InitMap() {

  std::cout << "size: " << map_data_.size[0] << std::endl;

  map_true_ = cv::Mat(cv::Size(map_data_.size[0],map_data_.size[1]),CV_32FC3);
  map_est_ = cv::Mat(cv::Size(map_data_.size[0],map_data_.size[1]),CV_32FC1);;
  map_est_.setTo(cv::Scalar::all(0.5));

  // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
  // // cv::imshow( "Display window", map_est_ );                   // Show our image inside it.

  // cv::waitKey(0); 

  cv::Mat image;
  cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
  cv::imshow("Display Image", image);
  cv::waitKey(0);

}