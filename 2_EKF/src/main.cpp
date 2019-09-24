#include "EKF.h"

#include <sstream> // for std::stringstream

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
EKF ekf(num_landmarks);

ekf.sim();


}