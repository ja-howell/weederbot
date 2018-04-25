cmake_minimum_required(VERSION 2.8.3)
project(husky_marker_tracking)

find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             message_generation
             image_transport
             cv_bridge
             tf
             aruco
             camera_calibration_parsers
             )

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)

include_directories(${PROJECT_SOURCE_DIR}/src/)


SET(SOURCES ${PROJECT_SOURCE_DIR}/src/husky_marker_tracking.cpp
            ${PROJECT_SOURCE_DIR}/src/aruco_detector.cpp)

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/aruco_detector.h)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} husky_marker_tracking )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${aruco_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES})
