cmake_minimum_required(VERSION 3.2.0
 FATAL_ERROR)
project(target_tracking)

find_package(catkin REQUIRED COMPONENTS
  axis_camera
  image_transport
  cv_bridge
  roscpp
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES target_tracking
#  CATKIN_DEPENDS axis_camera openCV roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(target_tracker src/main.cpp)
target_link_libraries(target_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_compile_features(target_tracker PUBLIC cxx_lambdas)
