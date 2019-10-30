
// #include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>

#include <stdio.h>
#include <string.h> /* For memcpy() */
#include <stdlib.h> /* For EXIT_FAILURE, EXIT_SUCCESS */
#include <iostream>
#include <color.h>
// #include "mat.h"
// #include "matrix.h"
// #include <Eigen/Core>
// #include <Eigen/LU>
// #include <Eigen/Dense>
// #include <math.h> 

const float kPi=3.14159;

struct MATLAB_DATA {

size_t num_elements;   // Total number of elements
size_t num_dims;       // Total number of matrix dimension
std::vector<double> data;          // Contains the data
std::vector<int> size;    // Size of each dimension (row, col, etc) 

double GetDataPoint(unsigned row, unsigned col, unsigned width)
{
  if (num_dims != 3)
  {
    // std::cerr << "The data is not three dimensional";
    return GetDataPoint(row,col);
  }
  else
  {
    int index = row + col*size[0]+width*size[0]*size[1];
    return data[index];
  }
}

double GetDataPoint(unsigned row, unsigned col) {
  if (num_dims != 2)
  {
    // std::cerr << "The data is not two dimensional";
    return NAN;
  }
  else
  {
    int index = row + col*size[0];
    // std::cout <<"index: " << index <<std::endl;
    return data[index];
  }
}

};

class ENV {


public:

ENV();
~ENV();

void SimMap();

private:

void InitMatlabData();
void InitMap();
void DisplayEstImg();
void UpdateMap(int time_step, int meas_number);
float InverseSensorModel(int time_step, int meas_number, int mapx, int mapy, float map_probability); 

MATLAB_DATA map_data_;
MATLAB_DATA X_data_;
MATLAB_DATA z_data_;
MATLAB_DATA thk_data_;

cv::Mat map_true_;
cv::Mat map_est_;

std::string true_image_ = "Display true";
std::string est_image_ = "Display est";

Color color_;

// Inverse model parameters
float alpha_ = 1;
float beta_ = 2.0/180.0*kPi;
float z_max_ = 150.0;
float locc_ = -0.3;     // log occupied 
float lfree_ = 0.3;    // log unoccupied




};