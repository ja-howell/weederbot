// Other
#include <vector>
#include <string>
// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
// Aruco libraries
// #include <aruco/aruco.h>
// #include <aruco/cameraparameters.h>
// #include <aruco/cvdrawingutils.h>
// #include <aruco/arucofidmarkers.h>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
// Transforms
#include <tf/transform_listener.h>

#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

typedef struct {
  cv::Mat Rvec;
  cv::Mat Tvec;
  int id;

} Marker;

class ArucoDetector {
public:
ArucoDetector(float marker_size);
tf::Transform arucoMarker2TransForm(const Marker &marker);
bool cameraParamsAreSet();
void setParams(cv::Mat CameraMatrix, cv::Mat DistortionMatrix, cv::Size size);
std::vector<Marker> getMarkersInView(cv::Mat image);

private:
float marker_size;
bool has_camera_params = false;
std::string reference_frame = "base_link";
std::string camera_frame = "axis_base";
tf::StampedTransform tf_to_base_link;
geometry_msgs::Pose current_camera_pose;
tf::Transform current_camera_tf;
cv::Mat CameraMatrix;
cv::Mat DistortionMatrix;
cv::Size size;
cv::Ptr<cv::aruco::Dictionary> dictionary;
cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
};
#endif
