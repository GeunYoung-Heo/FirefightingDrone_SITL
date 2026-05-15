cmake_minimum_required(VERSION 3.10)
project(disturbance_plugin CXX)

# Gazebo Classic 11 expects C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

include_directories(${GAZEBO_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${JSONCPP_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(disturbance_plugin SHARED disturbance_plugin.cc)
target_link_libraries(disturbance_plugin
  ${GAZEBO_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)
