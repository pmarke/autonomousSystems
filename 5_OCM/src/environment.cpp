#include "environment.h"



ENV::ENV() {

  InitMatlabData();
  InitMap();
  DisplayEstImg();

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

  // std::cerr << "starting to write data" << std::endl;

  double current;
  char data[sizeof(current)];


  while (!thkFile.eof()) {
    thkFile.read(data, sizeof(double));
    memcpy(&current, data,sizeof(double));    
    if(thkFile.eof()) break;
    thk_data_.data.push_back(current);
  }
  thk_data_.num_elements=thk_data_.data.size();
  thk_data_.num_dims = 2;
  thk_data_.size.push_back(1);
  thk_data_.size.push_back(9);

  // std::cerr << "starting to write state data" << std::endl;


  while (!stateFile.eof()) {
    stateFile.read(data, sizeof(double));
    memcpy(&current, data,sizeof(double));    
    if(stateFile.eof()) break;
    X_data_.data.push_back(current);
  }
  X_data_.num_elements=X_data_.data.size();
  X_data_.num_dims = 2;
  X_data_.size.push_back(3);
  X_data_.size.push_back(380);


  // std::cerr << "starting to write meas data" << std::endl;


  while (!measFile.eof()) {
    measFile.read(data, sizeof(double));
    memcpy(&current, data,sizeof(double));    
    if(measFile.eof()) break;
    z_data_.data.push_back(current);
  }
  z_data_.num_elements=z_data_.data.size();
  z_data_.num_dims = 3;
  z_data_.size.push_back(2);
  z_data_.size.push_back(9);
  z_data_.size.push_back(380);

  // std::cerr << "starting to write map data" << std::endl;


  while (!mapFile.eof()) {
    mapFile.read(data, sizeof(double));
    memcpy(&current, data,sizeof(double));    
    if(mapFile.eof()) break;
    map_data_.data.push_back(current);
  }
  map_data_.num_elements=map_data_.data.size();
  map_data_.num_dims = 2;
  map_data_.size.push_back(100);
  map_data_.size.push_back(100);

  // std::cout << map_data_.data.size() << std::endl;
  // std::cout << map_data_.GetDataPoint(99,99) << std::endl;

  // std::cerr << "finished writing data" << std::endl;



  thkFile.close();
  stateFile.close();
  measFile.close();
  mapFile.close();

}



//-----------------------------------------------------------------------------

void ENV::InitMap() {

  // std::cout << "size: " << map_data_.size[0] << std::endl;

  map_true_ = cv::Mat(map_data_.size[0],map_data_.size[1],CV_8UC3,cv::Scalar(0,0,0));
  // map_true_ = cv::Mat(100,100,CV_8UC3,cv::Scalar(255,255,255));
  map_est_ = cv::Mat(cv::Size(map_data_.size[0],map_data_.size[1]),CV_32FC1);;
  map_est_.setTo(cv::Scalar::all(0.5));

  // std::cout << "size: " <<map_true_.channels() << std::endl;

  map_true_.at<cv::Vec3b>(99,99) = color_.firebrick;
  // std::cout << "20 1: " << map_data_.GetDataPoint(0,20) << std::endl;

  for (int ii=0; ii < map_data_.size[0]; ii++) {
    // std::cout << "ii: " << ii << std::endl;
    for (int jj=0; jj < map_data_.size[1]; jj++) {
      if (map_data_.GetDataPoint(ii,jj)==1)
        map_true_.at<cv::Vec3b>(99-jj,ii) = color_.black;
      else
        map_true_.at<cv::Vec3b>(99-jj,ii) = color_.white;
    }
  }


  for (int ii=0; ii < X_data_.size[1]; ii++) {
    int x = static_cast<int>(X_data_.GetDataPoint(0,ii));
    int y = static_cast<int>(X_data_.GetDataPoint(1,ii));
    map_true_.at<cv::Vec3b>(99-y,x) = color_.blue;
  }



  cv::namedWindow( true_image_, cv::WINDOW_AUTOSIZE );
  // cv::resizeWindow(true_image_, 500, 500);
  cv::namedWindow(est_image_, cv::WINDOW_AUTOSIZE);
  cv::Mat ti_big;
  cv::resize(map_true_,ti_big,cv::Size(500,500));
  cv::imshow( true_image_, ti_big );

  // cv::waitKey(0);                   // Show our image inside it.



}

//-----------------------------------------------------------------------------

void ENV::DisplayEstImg() {

  cv::Mat img;
  cv::resize(map_est_,img,cv::Size(500,500));
  cv::imshow( est_image_, img );

  cv::waitKey(0);

}

//-----------------------------------------------------------------------------

void ENV::SimMap() {


// Loop through all of the time steps
for (int ii=0; ii < z_data_.size[2]; ii++) {
  // Loop through all of the measurements at each time step
  for (int jj=0; jj < z_data_.size[1]; jj++) {
    // If measurement isn't nan, processs measurement
    if (!std::isnan(z_data_.data))
      UpdateMap(ii, jj);


  }
}
}

//-----------------------------------------------------------------------------


void ENV::UpdateMap(int time_step, int meas_number) {

  // Loop through all map data
  for (int ii=0; ii < map_est_.size[0]; ii++) {
    for (int jj=0; jj < map_est_.size[1]; jj++) {
      
    }
  }

}

//-----------------------------------------------------------------------------

float ENV::InverseSensorModel(int time_step, int meas_number, int mapx, int mapy); 