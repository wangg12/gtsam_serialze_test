cmake_minimum_required(VERSION 3.20)
project(test_gtsam_serialize)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_CXX_FLAGS "-std=c++17 -O0" )


# Boost - same requirement as gtsam
find_package(Boost 1.65 REQUIRED)
find_package(GTSAM REQUIRED) # Uses installed package


include_directories(${Boost_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})


add_executable(test_gtsam_serialize test.cc)
target_link_libraries(test_gtsam_serialize gtsam ${Boost_LIBRARIES})