cmake_minimum_required(VERSION 3.0)
project(ekfSlam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



find_package (Eigen3 REQUIRED NO_MODULE)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/ekf_slam.cpp
)
# target_include_directories(${PROJECT_NAME}_lib
#   PRIVATE
#     ${EIGEN3_INCLUDE_DIR}
#   PUBLIC
#     include)

target_include_directories(${PROJECT_NAME}_lib    
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIR})



target_link_libraries(${PROJECT_NAME}_lib stdc++fs )

add_executable(main src/main.cpp)
target_link_libraries(main ${PROJECT_NAME}_lib)



