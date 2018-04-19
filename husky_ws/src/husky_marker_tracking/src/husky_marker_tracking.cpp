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

// Defines
#define CONTROLLER_Y_BUTTON 1
#define CONTROLLER_B_BUTTON 3
// Globals
cv::Mat current_image;
ros::Publisher roi_pub;
ros::Publisher camera_axis_pub;
ros::Subscriber controller_sub;
image_transport::Subscriber image_sub;
bool enabled = false;

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joymsg);
void imageCallback(const sensor_msgs::ImageConstPtr& imagemsg);

int main(int argc, char ** argv) {
        ROS_INFO("Waiting for Signal to start marker tracking");
        ros::init(argc, argv, "husky_marker_tracking");
        ros::NodeHandle nh;
        ros::Rate loop_rate(2);
        image_transport::ImageTransport transport(nh);

        roi_pub = nh.advertise<sensor_msgs::RegionOfInterest>("/weederbot/grid", 1);
        camera_axis_pub = nh.advertise<axis_camera::Axis>("/axis/cmd", 1);
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

        while(ros::ok()) {
                if(enabled) {
                        //    Check for markers in current view
                        //    If non in view pan
                        //    else add to list
                        //    pan for next marker
                        //    when done publish
                }
                ros::spinOnce();
                loop_rate.sleep();
        }

        return 0;
}

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joymsg) {
        if(!enabled) {
                if(joymsg->buttons[CONTROLLER_Y_BUTTON] == 1 && joymsg->buttons[CONTROLLER_B_BUTTON] == 1) {
                        enabled = true;
                }
        }
}

void imageCallback(const sensor_msgs::ImageConstPtr& imagemsg) {
        try {
                cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(imagemsg, "rgb8");
                current_image = img_ptr->image;
        }catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
        }
}
