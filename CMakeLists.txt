cmake_minimum_required(VERSION 2.8)
project(stereo_camera_vo)

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -O0 -Wall -ggdb -fopenmp -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# fmt
find_package(fmt REQUIRED)
include_directories(${fmt_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# yaml
find_package(yaml-cpp REQUIRED)

set(
  THIRD_PARTY_LIBS
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${Pangolin_LIBRARIES} GL GLU GLEW glut
  g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
  pthread
  ${CSPARSE_LIBRARY}
  fmt::fmt
  yaml-cpp
)


add_subdirectory(src)
