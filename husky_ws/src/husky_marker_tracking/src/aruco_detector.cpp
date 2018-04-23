#include "aruco_detector.h"

ArucoDetector::ArucoDetector(float marker_size) {
        this->marker_size = marker_size;
}

tf::Transform ArucoDetector::arucoMarker2TransForm(const aruco::Marker &marker) {
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

void ArucoDetector::setParams(aruco::CameraParameters params) {
        this->cameraParams = params;
        this->has_camera_params = true;
}

std::vector<aruco::Marker> ArucoDetector::getMarkersInView(cv::Mat image) {
        std::vector<aruco::Marker> markers;
        do {
                // Get the Markers in the Image using ROS Aruco
                this->detector.detect(image, markers, this->cameraParams, this->marker_size);
                if(markers.size() < 1) {
                        ROS_DEBUG("Could not find any Markers");
                        // Start "Panning" for markers
                }
        } while(markers.size() < 1);

        return markers;
}
