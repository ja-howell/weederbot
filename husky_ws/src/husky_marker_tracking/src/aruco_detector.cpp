#include "aruco_detector.h"

ArucoDetector::ArucoDetector(float marker_size, int marker_dictionary_id) {
        this->marker_size = marker_size;
        this->marker_dictionary_id = marker_dictionary_id;
        this->dictionary = cv::aruco::getPredefinedDictionary(
                cv::aruco::PREDEFINED_DICTIONARY_NAME(this->marker_dictionary_id));
        this->detectorParams = cv::aruco::DetectorParameters();
}

std::vector<int> ArucoDetector::markersInView(cv::Mat image) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;
        cv::Mat img;
        cv::aruco::detectMarkers(img, this->dictionary, corners, ids, this->detectorParams, rejected);
        return ids;
}

std::vector<tf::Transform> ArucoDetector::getMarkerTransforms(cv::Mat image, cv::Mat CameraMatrix, cv::Mat DistortionMatrix, cv::Size size) {
        std::vector<tf::Transform> transforms;
        tf::StampedTransform cameraToReference;
        cameraToReference.setIdentity();
        getTransform(reference_frame, camera_frame, cameraToReference);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;
        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::detectMarkers(image, this->dictionary, corners, ids, this->detectorParams, rejected);

        cv::aruco::estimatePoseSingleMarkers(corners, this->marker_size, CameraMatrix, DistortionMatrix,rvecs, tvecs);
        for (size_t i = 0; i < rvecs.size(); i++) {
                tf::Transform markerTransform = this->ConvertToTransform(rvecs[i], tvecs[i]);
                transforms.push_back(markerTransform);
        }

        return transforms;
}

bool ArucoDetector::getTransform(const std::string& refFrame,
                                 const std::string& childFrame,
                                 tf::StampedTransform& transform)
{
        std::string errMsg;

        if(!tfListener_.waitForTransform(refFrame,
                                         childFrame,
                                         ros::Time(0),
                                         ros::Duration(0.5),
                                         ros::Duration(0.01),
                                         &errMsg))
        {
                ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
                return false;
        }
        else
        {
                try
                {
                        tfListener_.lookupTransform(refFrame, childFrame,
                                                    ros::Time(0), //get latest available
                                                    transform);
                }
                catch ( const tf::TransformException& e)
                {
                        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
                        return false;
                }

        }
        return true;
}

tf::Transform ArucoDetector::ConvertToTransform(cv::Vec3d rvec, cv::Vec3d tvec) {
        cv::Mat Rvec(rvec, true);
        cv::Mat Tvec(tvec, true);
        cv::Mat rot(3, 3, CV_64FC1);
        cv::Mat Rvec64;
        Rvec.convertTo(Rvec64, CV_64FC1);
        cv::Rodrigues(Rvec64, rot);
        cv::Mat tran64;
        Tvec.convertTo(tran64, CV_64FC1);

        cv::Mat rotate_to_ros(3, 3, CV_64FC1);
        rotate_to_ros.at<double>(0,0) = -1.0;
        rotate_to_ros.at<double>(0,1) = 0.0;
        rotate_to_ros.at<double>(0,2) = 0.0;
        rotate_to_ros.at<double>(1,0) = 0.0;
        rotate_to_ros.at<double>(1,1) = 0.0;
        rotate_to_ros.at<double>(1,2) = 1.0;
        rotate_to_ros.at<double>(2,0) = 0.0;
        rotate_to_ros.at<double>(2,1) = 1.0;
        rotate_to_ros.at<double>(2,2) = 0.0;
        rot = rot*rotate_to_ros.t();

        tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                             rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                             rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

        tf::Vector3 tf_orig(tran64.at<double>(0,0), tran64.at<double>(1,0), tran64.at<double>(2,0));


        return tf::Transform(tf_rot, tf_orig);
}
