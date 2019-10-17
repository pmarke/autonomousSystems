#include <stdio.h>
#include <string.h> /* For memcpy() */
#include <stdlib.h> /* For EXIT_FAILURE, EXIT_SUCCESS */
#include <iostream>
#include "mat.h"
#include "matrix.h"
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <math.h> 
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


struct MATLAB_DATA {

size_t num_elements;   // Total number of elements
size_t num_dims;       // Total number of matrix dimension
double* data;          // Contains the data
size_t* size;    // Size of each dimension (row, col, etc) 

double GetDataPoint(unsigned row, unsigned col, unsigned width)
{
  if (num_dims != 3)
  {
    std::cerr << "The data is not three dimensional";
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
    std::cerr << "The data is not two dimensional";
    return NAN;
  }
  else
  {
    int index = row + col*size[0];
    return data[index];
  }
}

~MATLAB_DATA()
{
  delete [] data;
  delete [] size;
}

};

class ENV {


public:

ENV(const char* matlab_file_name);
~ENV();

private:

void InitMatlabData(const char* matlab_file_name);
void InitMap();
void CreateMatlabStruct(MATFile* pmat, const char* var_name, MATLAB_DATA& mat_struct);

MATLAB_DATA map_data_;
MATLAB_DATA X_data_;
MATLAB_DATA z_data_;
MATLAB_DATA thk_data_;

cv::Mat map_true_;
cv::Mat map_est_;




};