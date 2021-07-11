cmake_minimum_required(VERSION 3.13)
project(aso)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP)

include_directories(../../..)

add_executable(computeASO main.cpp)
if(OPENMP_FOUND)
  target_link_libraries(computeASO Eigen3::Eigen OpenMP::OpenMP_CXX)
else()
  target_link_libraries(computeASO Eigen3::Eigen)
endif()
