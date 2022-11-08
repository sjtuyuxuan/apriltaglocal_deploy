cmake_minimum_required(VERSION 3.0.2)
find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")
find_package (
        yaml-cpp REQUIRED
        PATHS /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp
        NO_DEFAULT_PATH
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4)
find_package(Ceres REQUIRED)
project(localization)
include_directories(
        ${YAML_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${apriltag_INCLUDE_DIRS}
)
set(CMAKE_CXX_STANDARD 14)

add_executable(localization main.cpp)
target_link_libraries(localization ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} ${CERES_LIBRARIES} ${apriltag_LIBRARIES})

