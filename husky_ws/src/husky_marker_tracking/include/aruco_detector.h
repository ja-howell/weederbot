// Aruco libraries
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/arucofidmarkers.h>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H
class ArucoDetector {
public:
  ArucoDetector(float marker_size, int marker_dictionary_id);
private:
  float marker_size;
  int marker_dictionary_id;
};
#endif
