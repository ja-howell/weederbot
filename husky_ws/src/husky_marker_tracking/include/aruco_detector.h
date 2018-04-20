// Other
#include <vector>
#include <string>
// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
// Aruco libraries
#include <opencv2/aruco.hpp>
// Transforms
#include <tf/transform_listener.h>

#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H
class ArucoDetector {
public:
ArucoDetector(float marker_size, int marker_dictionary_id);
std::vector<int> markersInView(cv::Mat image);
std::vector<tf::Transform> getMarkerTransforms(cv::Mat image,
                                                      cv::Mat CameraMatrix,
                                                      cv::Mat DistortionMatrix,
                                                      cv::Size size);
bool getTransform(const std::string& refFrame,
                  const std::string& childFrame,
                  tf::StampedTransform& transform);
private:
float marker_size;
int marker_dictionary_id;
std::string reference_frame = "base_link";
std::string camera_frame = "axis_base";
cv::aruco::Dictionary dictionary;
cv::aruco::DetectorParameters detectorParams;
tf::TransformListener tfListener_;
tf::Transform ConvertToTransform(cv::Vec3d rvec, cv::Vec3d tvec);
};
#endif
