#include "ekfslam/ekf_slam.h"

#include <sstream> // for std::stringstream

int main(int argc, char **argv)
{

int num_landmarks = 3;
float radius = 10;
float vel = 1;
if (argc > 1)
{
	std::stringstream convert(argv[1]);
	
	if (!(convert >> num_landmarks))
	{
		num_landmarks = 3;
		std::cout << "Did not convert:" << std::endl;
	}
}
if (argc > 2)
{
  std::stringstream convert(argv[2]);
  
  if (!(convert >> vel))
  {
    vel=1;
    std::cout << "Did not convert:" << std::endl;
  }
}
if (argc > 3)
{
  std::stringstream convert(argv[3]);
  
  if (!(convert >> radius))
  {
    radius = 10;
    std::cout << "Did not convert:" << std::endl;
  }
}

std::cout << "num landmarks: " << num_landmarks << std::endl;
EKF_SLAM ekfs(num_landmarks,vel,radius);

ekfs.Sim();


}