#include "fastSlam/fast_slam.h"

#include <sstream> // for std::stringstream

int main(int argc, char **argv)
{

int num_landmarks = 3;
float radius = 10;
float fov = 2*3.14159;
float vel = 1;
int num_particles = 1000;
if (argc > 1)
{
  std::stringstream convert(argv[1]);
  
  if (!(convert >> num_landmarks))
  {
    num_landmarks = 3;
    std::cout << "Num Landmarks Did not convert:" << std::endl;
  }
}
if (argc > 2)
{
  std::stringstream convert(argv[2]);
  
  if (!(convert >> fov))
  {
    fov=2*3.14159;
    std::cout << "FOV Did not convert:" << std::endl;
  }
}
if (argc > 3)
{
  std::stringstream convert(argv[3]);
  
  if (!(convert >> num_particles))
  {
    num_particles=1000;
    std::cout << "Num Particles Did not convert:" << std::endl;
  }
}

if (argc > 4)
{
  std::stringstream convert(argv[4]);
  
  if (!(convert >> vel))
  {
    vel=1;
    std::cout << "Vel Did not convert:" << std::endl;
  }
}
if (argc > 5)
{
  std::stringstream convert(argv[5]);
  
  if (!(convert >> radius))
  {
    radius = 10;
    std::cout << "Radius Did not convert:" << std::endl;
  }
}

std::cout << "num landmarks: " << num_landmarks << std::endl;
std::cout << "FOV: " << fov << std::endl;
std::cout << "num particles: " << num_particles << std::endl;
// std::cout << "num landmarks: " << num_landmarks << std::endl;
FastSlam fast_slam(num_landmarks,fov,num_particles,vel,radius);
fast_slam.Sim();



}