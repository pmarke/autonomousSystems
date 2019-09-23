cmake_minimum_required(VERSION 2.8.3)
project(extended_kalman_filter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



find_package (Eigen3 REQUIRED NO_MODULE)


include_directories(
include
${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/EKF.cpp
)

target_link_libraries(${PROJECT_NAME}_lib stdc++fs )

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME}_lib)
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_kalman_filter)



