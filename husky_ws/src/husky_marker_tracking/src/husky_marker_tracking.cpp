#include "ros/ros.h"
#include <vector>
#include <iostream>

// Aruco Detector
#include "aruco_detector.h"
// Axis Camera
#include <axis_camera/Axis.h>

// Joystick
#include <sensor_msgs/Joy.h>

// ROI
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Defines
#define CONTROLLER_Y_BUTTON 1
#define CONTROLLER_B_BUTTON 3
// Globals
cv::Mat current_image;
cv::Mat CameraMatrix;
cv::Size size;
cv::Mat DistortionMatrix;
ros::Publisher roi_pub;
ros::Publisher camera_axis_pub;
ros::Subscriber controller_sub;
ros::Subscriber camera_info_sub;
tf::Transform base_link_transform;
image_transport::Subscriber image_sub;
bool enabled = false;
const float marker_size = 0.0325; // meters
const int marker_dictionary_id = 0;

// Aruco Detector
ArucoDetector detector(marker_size);

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joymsg);
void imageCallback(const sensor_msgs::ImageConstPtr& imagemsg);
void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camInfoMsg);

int main(int argc, char ** argv) {
        ROS_INFO("Waiting for Signal to start marker tracking");
        ros::init(argc, argv, "husky_marker_tracking");
        ros::NodeHandle nh;
        ros::Rate loop_rate(2);
        image_transport::ImageTransport transport(nh);

        // Publishers and Subscribers
        roi_pub = nh.advertise<sensor_msgs::RegionOfInterest>("/weederbot/grid", 1);
        camera_axis_pub = nh.advertise<axis_camera::Axis>("/axis/cmd", 1);
        camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/axis/camerainfo", 10, cameraInfoCallback);
        controller_sub = nh.subscribe<sensor_msgs::Joy>("/joy_teleop/joy", 10, joystickCallback);
        image_sub = transport.subscribe("/axis/image_raw_out", 1, imageCallback);

        // axis msgs
        axis_camera::Axis axis_msg;
        axis_msg.pan = 0;
        axis_msg.tilt = 0;
        axis_msg.zoom = 0;
        axis_msg.brightness = 5000;
        axis_msg.autofocus = true;

        camera_axis_pub.publish(axis_msg);

        int current_marker = 0;

        while(ros::ok()) {
                float length = 0, width = 0;
                if(enabled && detector.cameraParamsAreSet()) {
                        // Get the Markers
                        std::vector<aruco::Marker> markers = detector.getMarkersInView(current_image);
                        // Get the 'First' Marker
                        aruco::Marker marker = markers[0];
                        // Get the marker pose
                        tf::Transform marker_transform = detector.arucoMarker2TransForm(marker);
                        // Publish the Marker Pose
                        // Publish the Marker Transform to the Camera
                }
                ros::spinOnce();
                loop_rate.sleep();
        }

        return 0;
}

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joymsg) {
        if(!enabled) {
                if(joymsg->buttons[CONTROLLER_Y_BUTTON] == 1 && joymsg->buttons[CONTROLLER_B_BUTTON] == 1) {
                        ROS_INFO("Mowing Enabled");
                        enabled = true;
                }
        }
}

void imageCallback(const sensor_msgs::ImageConstPtr& imagemsg) {
        try {
                current_image = cv_bridge::toCvShare(imagemsg, "bgr8")->image;
        }catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
        }
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camInfoMsg) {
        CameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
        DistortionMatrix = cv::Mat::zeros(4, 1, CV_64FC1);
        size = cv::Size(camInfoMsg->height, camInfoMsg->width);

        for (size_t i = 0; i < 9; ++i) {
                CameraMatrix.at<double>(i%3, i-(i%3)*3) = camInfoMsg->K[i];
        }

        if(camInfoMsg->D.size() == 4) {
                for (size_t i = 0; i < 4; ++i) {
                        DistortionMatrix.at<double>(i, 0) = camInfoMsg->D[i];
                }
        }

        aruco::CameraParameters params;
        params.setParams(CameraMatrix, DistortionMatrix, size);


}
