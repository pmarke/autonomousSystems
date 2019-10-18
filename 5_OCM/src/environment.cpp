#include "environment.h"



ENV::ENV() {

  InitMatlabData();
  InitMap();
  // DisplayEstImg();

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
  thk_data_.size.push_back(11);

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
  X_data_.size.push_back(759);


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
  z_data_.size.push_back(11);
  z_data_.size.push_back(759);

  std::cerr << "starting to write map data" << std::endl;


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

  std::cerr << "finished writing data" << std::endl;



  thkFile.close();
  stateFile.close();
  measFile.close();
  mapFile.close();

}



//-----------------------------------------------------------------------------

void ENV::InitMap() {

  std::cout << "size: " << map_data_.size[0] << std::endl;
  std::cout << "size: " << map_data_.size[1] << std::endl;
  std::cout << "size d: " << map_data_.data.size() << std::endl;

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

  std::cout << "size: " << X_data_.size[1] << std::endl;

  for (int ii=0; ii < X_data_.size[1]; ii++) {
    int x = static_cast<int>(X_data_.GetDataPoint(0,ii));
    int y = static_cast<int>(X_data_.GetDataPoint(1,ii));
    map_true_.at<cv::Vec3b>(99-y,x) = color_.blue;
  }



  cv::namedWindow( true_image_, cv::WINDOW_AUTOSIZE );
  // cv::resizeWindow(true_image_, 500, 500);
  cv::namedWindow(est_image_, cv::WINDOW_NORMAL);
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


  // std::cerr << "z data size 2: " << z_data_.size[2] << std::endl;
  // std::cerr << "z data size 1: " << z_data_.size[1] << std::endl;

  // Loop through all of the time steps
  for (int time_step=0; time_step < z_data_.size[2]; time_step++) {
    // Loop through all of the measurements at each time step
    for (int meas_num=0; meas_num < z_data_.size[1]; meas_num++) {
      // If measurement isn't nan, processs measurement
      if (!std::isnan(z_data_.GetDataPoint(0,meas_num,time_step)))
      {
        UpdateMap(time_step, meas_num);
        // DisplayEstImg();
      }
    }
  }

DisplayEstImg();

}

//-----------------------------------------------------------------------------


void ENV::UpdateMap(int time_step, int meas_number) {

  // Loop through all map data
  for (int ii=0; ii < map_est_.size[0]; ii++) {
    for (int jj=0; jj < map_est_.size[1]; jj++) {
      float map_probability = map_est_.at<float>(ii,jj);
      int mapx = jj+1;
      int mapy = 99-ii+1;
      map_est_.at<float>(ii,jj) = InverseSensorModel(time_step,meas_number,mapx,mapy,map_probability);
      
    }
  }

}

//-----------------------------------------------------------------------------

float ENV::InverseSensorModel(int time_step, int meas_number, int mapx, int mapy, float map_probability) {

  // Robot's x,y pos and heading
  float rx = X_data_.GetDataPoint(0,time_step);
  float ry = X_data_.GetDataPoint(1,time_step);
  float rth = X_data_.GetDataPoint(2,time_step);

  // Measurement range and angle
  float zr = z_data_.GetDataPoint(0,meas_number,time_step);
  float zth = z_data_.GetDataPoint(1,meas_number,time_step);

  // Compute the range and heading to the grid cell
  float r = sqrt(pow(mapx-rx,2)+pow(mapy-ry,2));
  float th = atan2(mapy-ry,mapx-rx)-rth;

  // wrap theta
  if (th > kPi)
    th = th-2*kPi;
  else if (th <= -kPi)
    th = th+2*kPi;




  float update = 0;

  // See if map location is within this cone shape
  if (r > std::min(z_max_, zr+alpha_/2) || std::fabs(th-zth)>beta_/2)
    update = 0;

  // See if map is on the edge of the cone
  else if (zr < z_max_  && fabs(r-zr) < alpha_/2)
    update = locc_;
  else
    update = lfree_;

  // convert from log to probability
  if (update == 0)
    update = map_probability;
  else {
    float ldl = log(map_probability/(1.0f-map_probability));
    update += ldl;
    update = 1.0f-1.0f/(1.0f+exp(update));
  }


// && update !=map_probability
    if (false  && meas_number ==8 & mapx ==0) {
    std::cerr << std::endl;
    std::cerr << "rx: " << rx << std::endl;
    std::cerr << "ry: " << ry << std::endl;
    std::cerr << "rth: " << rth << std::endl;
    std::cerr << "zr: " << zr << std::endl;
    std::cerr << "zth: " << zth << std::endl;
    std::cerr << "r: " << r << std::endl;
    std::cerr << "th: " << th << std::endl;
    std::cerr << "mapx: " << mapx << std::endl;
    std::cerr << "mapy: " << mapy << std::endl;
  }

  // if (r > std::min(z_max_, zr+alpha_/2) || std::fabs(th-zth)>beta_/2)
  //   update = 0;
  // else if (zr < z_max_ && fabs(th-zth) < alpha_/2)
  //   update = 0.9;
  // else
  //   update = 0;


  return update;


}