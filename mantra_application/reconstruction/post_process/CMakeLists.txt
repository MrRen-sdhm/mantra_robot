cmake_minimum_required(VERSION 3.10)
project(post_process)
add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Opencv
find_package(OpenCV 3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -std=c++14")

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Eigen library
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(extract_normals extract_normals.cpp)
target_link_libraries(extract_normals
        ${PCL_LIBRARIES}
        ${EIGEN_LIBRARIES})


add_executable(create_point_cloud create_point_cloud.cpp)
target_link_libraries(create_point_cloud
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES})
