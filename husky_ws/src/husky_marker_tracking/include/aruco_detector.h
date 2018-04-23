// Other
#include <vector>
#include <string>
// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
// Aruco libraries
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>
#include <aruco/cvdrawingutils.h>
#include <aruco/arucofidmarkers.h>
// Transforms
#include <tf/transform_listener.h>

#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H
class ArucoDetector {
public:
ArucoDetector(float marker_size);
tf::Transform arucoMarker2TransForm(const aruco::Marker &marker);
bool cameraParamsAreSet();
void setParams(aruco::CameraParameters params);
std::vector<aruco::Marker> getMarkersInView(cv::Mat image);

private:
float marker_size;
bool has_camera_params = false;
aruco::MarkerDetector detector;
aruco::CameraParameters cameraParams;
std::string reference_frame = "base_link";
std::string camera_frame = "axis_base";
tf::StampedTransform tf_to_base_link;
geometry_msgs::Pose current_camera_pose;
tf::Transform current_camera_tf;

};
#endif
