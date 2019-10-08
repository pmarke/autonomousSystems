#include "simulation.h"


int main(int argc, char **argv)
{

int num_landmarks = 3;
if (argc ==2)
{
  std::stringstream convert(argv[1]);
  
  if (!(convert >> num_landmarks))
  {
    num_landmarks = 3;
    std::cout << "Did not convert:" << std::endl;
  }
}

std::cout << "num landmarks: " << num_landmarks << std::endl;
Sim sim(num_landmarks,1000);
sim.Simulate();



}