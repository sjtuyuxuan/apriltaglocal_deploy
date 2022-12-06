cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a.so")
set(BUILD_SHARED_LIBS OFF)
# set(CMAKE_EXE_LINKER_FLAGS "-static")
set(GLOG_LIBRARY "/usr/lib/arm-linux-gnueabihf/cmake/glog")
find_package(gflags REQUIRED PATHS /usr/lib/arm-linux-gnueabihf/cmake/gflags)
find_package(glog REQUIRED PATHS /usr/lib/arm-linux-gnueabihf/cmake/glog)
find_package(PkgConfig)
# find_package(apriltag REQUIRED PATHS /usr/local/share/apriltag)
pkg_search_module(apriltag REQUIRED apriltag)
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")
find_package (
        yaml-cpp REQUIRED
        PATHS /usr/lib/arm-linux-gnueabihf/cmake/yaml-cpp
        NO_DEFAULT_PATH
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED PATHS /usr/lib/arm-linux-gnueabihf/cmake/opencv4)
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

