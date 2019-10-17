

// #include <stdio.h>
// #include <string.h> /* For memcpy() */
// #include <stdlib.h> /* For EXIT_FAILURE, EXIT_SUCCESS */
// #include <iostream>
// #include "mat.h"
// #include "matrix.h"
// #include <Eigen/Core>
// #include <Eigen/LU>
// #include <Eigen/Dense>
// #include <math.h>       /* isnan, sqrt */
#include "environment.h"



int main(int argn, char** argv)
{



  // MATFile *pmat;
  const char *myFile = "/home/mark/projects/autonomousSystems/5_OCM/matlab/state_meas_data.mat";

  ENV env(myFile);

  // pmat = matOpen(myFile,"r");
  // if (pmat == nullptr)
  //   std::cerr << "file not found" << std::endl;

  // mxArray *meas, *state, *thk, *map;

  // meas = matGetVariable(pmat,"z");
  // state = matGetVariable(pmat,"X");
  // thk = matGetVariable(pmat,"thk");

  // // Get sizes
  // const mwSize *meas_size =  mxGetDimensions(meas);
  // const mwSize *state_size =  mxGetDimensions(state);
  // const mwSize *thk_size =  mxGetDimensions(thk);
  // const size_t meas_dims = mxGetNumberOfDimensions(meas);
  // const size_t state_dims = mxGetNumberOfDimensions(state);
  // const size_t thk_dims = mxGetNumberOfDimensions(thk);
  // size_t meas_num_elements = mxGetNumberOfElements(meas);
  // size_t state_num_elements = mxGetNumberOfElements(state);
  // size_t thk_num_elements = mxGetNumberOfElements(thk);

  // mxDouble* thk_data = mxGetDoubles(thk);
  // mxDouble* meas_data = mxGetDoubles(meas);
  // mxDouble* state_data = mxGetDoubles(state);

  // std::cout << "meas num elements: " << meas_num_elements << std::endl;
  // std::cout << "state num elements: " << state_num_elements << std::endl;
  // std::cout << "thk num elements: " << thk_num_elements << std::endl;


  // std::cout << "meas dims: " << meas_dims << std::endl;
  // std::cout << "state_dims dims: " << state_dims << std::endl;
  // std::cout << "thk_dims dims: " << thk_dims << std::endl;
 
  // std::cout << meas_size[0] << std::endl;
  // std::cout << meas_size[1] << std::endl;
  // std::cout << meas_size[2] << std::endl;

  // std::cout << thk_size[0] << std::endl;
  // std::cout << thk_size[1] << std::endl;  

  // std::cout << state_size[0] << std::endl;
  // std::cout << state_size[1] << std::endl;

  // // std::cout << "data" << thk_data[0] << std::endl;
  // // std::cout << "data" << std::isnan(meas_data[6840-12]) << std::endl;
  // // std::cout << "data" <<meas_data[0] << std::endl;
  // // std::cout << "data" <<meas_data[1] << std::endl;
  // // std::cout << "data" <<meas_data[2] << std::endl;
  // // std::cout << "data" <<meas_data[3] << std::endl;

  // // double* temp = meas_data;
  // double temp[meas_num_elements];
  // memcpy(temp,meas_data,meas_num_elements*8);
  // // std::cout << "data" <<temp[0] << std::endl;
  // // std::cout << "data" <<temp[1] << std::endl;
  // // std::cout << "data" <<temp[2] << std::endl;
  // // std::cout << "data" <<temp[3] << std::endl;
  // // std::cout << "data" << state_data[2] << std::endl;

  // // double test = new double[5] {1,2,3,4,5};

  // std::cout << "data " <<meas_data[1000] << std::endl;
  // std::cout << "data: " <<temp[1000] << std::endl;
  // std::cout << "data " <<meas_data[meas_num_elements-1] << std::endl;
  // std::cout << "data: " <<temp[meas_num_elements-1] << std::endl;
  // std::cout << "size: " << meas_size[0]*meas_size[1]*meas_size[2] << std::endl;
  
  // // meas_data = nullptr;
  
  // // std::cout << "data: " <<temp[0] << std::endl;
  // // delete [] temp;
  // // std::cout << "test: " << test[0] << std::endl;

  // // delete [] test;
  // // std::cout << "test: " << test[0] << std::endl;
  // // delete[meas_size[0]*meas_size[1]*meas_size[2]] temp;
  // // delete [] temp;
  // matClose(pmat);
  // mxDestroyArray(meas);
  // // delete [] test;
  // // mxFree(meas_data);
  // // free(meas_data);


  // // mxDestroyArray(state);
  // // mxDestroyArray(thk);

  return 0;
}