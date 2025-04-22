
cmake_minimum_required(VERSION 3.16.3)
project(data_creator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread")

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED DATACREATOR_VERSION)
  set(DATACREATOR_VERSION_MAJOR 1)
  set(DATACREATOR_VERSION_MINOR 0)
  set(DATACREATOR_VERSION_PATCH 0)
  set(DATACREATOR_VERSION ${DATACREATOR_VERSION_MAJOR}.${DATACREATOR_VERSION_MINOR}.${DATACREATOR_VERSION_PATCH})
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install/${PROJECT_NAME}-${DATACREATOR_VERSION})
endif()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(common_lib 
${PCL_LIBRARIES})

include(cmake/build_libraries.cmake)

include(cmake/build_test.cmake)

include(cmake/install_files.cmake)

add_executable(cylinder_app  src/sample/cylinder.cpp app/sample/cylinder_app.cpp src/sample/plane.cpp src/sample/ray.cpp)
target_include_directories(cylinder_app PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cylinder_app ${common_lib})