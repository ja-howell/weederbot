#include "ros/ros.h"
// Aruco libraries
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/arucofidmarkers.h>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char ** argv) {
  ros::init(argc, argv, "husky_marker_tracking");

  ros::NodeHandle nh;

  return 0;
}
