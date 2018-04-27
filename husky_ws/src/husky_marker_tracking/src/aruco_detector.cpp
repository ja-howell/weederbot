#include "aruco_detector.h"

ArucoDetector::ArucoDetector(float marker_size) {
        this->marker_size = marker_size;
        this->dictionary = cv::aruco::getPredefinedDictionary(
                cv::aruco::PREDEFINED_DICTIONARY_NAME(0));
        this->detectorParams = cv::aruco::DetectorParameters();
}

tf::Transform ArucoDetector::arucoMarker2TransForm(const Marker &marker) {
        cv::Mat marker_rotation(3,3, CV_32FC1);
        cv::Rodrigues(marker.Rvec, marker_rotation);
        cv::Mat marker_translation = marker.Tvec;

        cv::Mat rotate_to_ros(3,3,CV_32FC1);
        rotate_to_ros.at<float>(0,0) = -1.0;
        rotate_to_ros.at<float>(0,1) = 0;
        rotate_to_ros.at<float>(0,2) = 0;
        rotate_to_ros.at<float>(1,0) = 0;
        rotate_to_ros.at<float>(1,1) = 0;
        rotate_to_ros.at<float>(1,2) = 1.0;
        rotate_to_ros.at<float>(2,0) = 0.0;
        rotate_to_ros.at<float>(2,1) = 1.0;
        rotate_to_ros.at<float>(2,2) = 0.0;

        marker_rotation = marker_rotation * rotate_to_ros.t();

        tf::Matrix3x3 marker_tf_rot(marker_rotation.at<float>(0,0),marker_rotation.at<float>(0,1),marker_rotation.at<float>(0,2),
                                    marker_rotation.at<float>(1,0),marker_rotation.at<float>(1,1),marker_rotation.at<float>(1,2),
                                    marker_rotation.at<float>(2,0),marker_rotation.at<float>(2,1),marker_rotation.at<float>(2,2));

        tf::Vector3 marker_tf_tran(marker_translation.at<float>(0,0),
                                   marker_translation.at<float>(1,0),
                                   marker_translation.at<float>(2,0));

        return tf::Transform(marker_tf_rot, marker_tf_tran);
}

bool ArucoDetector::cameraParamsAreSet() {
        return this->has_camera_params;
}

void ArucoDetector::setParams(cv::Mat CameraMatrix, cv::Mat DistortionMatrix, cv::Size size) {
        this->CameraMatrix = CameraMatrix;
        this->DistortionMatrix = DistortionMatrix;
        this->size = size;
        this->has_camera_params = true;
}

std::vector<Marker> ArucoDetector::getMarkersInView(cv::Mat image) {
        std::vector<Marker> markers;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::detectMarkers(image, this->dictionary, corners, ids, detectorParams,rejected);
        if (ids.size() > 0) {
                cv::aruco::estimatePoseSingleMarkers(corners, this->marker_size,
                                                     this->CameraMatrix, this->DistortionMatrix,
                                                     rvecs, tvecs);
                for (size_t i = 0; i < ids.size(); i++) {
                  int id = ids.at(i);
                  cv::Vec3d rvec = rvecs.at(i);
                  cv::Vec3d tvec = tvecs.at(i);
                  Marker marker;
                  marker.id = id;
                  // marker.Rvec.create(3,1,CV_32FC1);
                  // marker.Tvec.create(3,1,CV_32FC1);
                  marker.Rvec = cv::Mat(rvec);
                  marker.Tvec = cv::Mat(tvec);
                  markers.push_back(marker);
                }
        }

        return markers;
}
