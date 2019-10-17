cmake_minimum_required(VERSION 2.8.3)
project(map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



find_package (Eigen3 REQUIRED NO_MODULE)

# find_package (Matlab REQUIRED )
find_package(OpenCV 3 REQUIRED)
# find_package(TIFF 4 REQUIRED)

include_directories(
include
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
# ${Matlab_ROOT_DIR}/extern/include

)


# set(OpenCV_STATIC ON)


# message(${Matlab_ROOT_DIR})


# ## Declare a C++ library
# add_library(env_lib
#   src/environment.cpp
# )


# add_library(sim_lib
#   src/simulation.cpp
# )

# target_link_libraries(${PROJECT_NAME}_lib stdc++fs )
# target_link_libraries(sim_lib stdc++fs ${PROJECT_NAME}_lib)
# add_dependencies(sim_lib ${PROJECT_NAME}_lib)



# if(UNIX)
#     message(STATUS "Linking UNIX C++")
#     # target_link_libraries(${PROJECT_NAME} ${Matlab_ROOT_DIR}/extern/bin/glnxa64/libMatlabEngine.so)
#     # target_link_libraries(${PROJECT_NAME} ${Matlab_ROOT_DIR}/extern/bin/glnxa64/libMatlabDataArray.so)
#     target_link_libraries(env_lib ${Matlab_ROOT_DIR}/bin/glnxa64/libmat.so)
#     target_link_libraries(env_lib ${Matlab_ROOT_DIR}/bin/glnxa64/libmx.so)
#     # find_package(Threads)
#     # target_link_libraries(env_lib ${CMAKE_THREAD_LIBS_INIT})
# endif(UNIX)
# # target_link_libraries(main sim_lib)

# target_link_libraries(env_lib ${OpenCV_LIBS})

add_executable(${PROJECT_NAME} src/main.cpp)
# target_link_libraries(${PROJECT_NAME} env_lib)
# add_dependencies(${PROJECT_NAME} env_lib)