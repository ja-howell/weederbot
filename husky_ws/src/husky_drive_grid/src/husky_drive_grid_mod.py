#!/usr/bin/env python

# Husky Drive Grid
# By Ryan Owens
# Subscribes to an 'area' measurement and sends move_base Actions to
# cover the area

import math
import rospy
import actionlib
from enum import Enum
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import RegionOfInterest
from tf.transformations import quaternion_from_euler, euler_from_quaternion


__ROBOT__WIDTH__ = 0.67 # Meters
__ROBOT__LENGTH__ = 0.99 # Meters
__ROW_OVERLAP__ = 0.4 # Meters
__ROBOT__BASE_LINK__ = "base_link"
__ROBOT__TURN_AROUND_LENGTH = 2 # Meters
__CMD_VEL_PUBLISHER = None
__CURRENT_POSE = None
__LAST_OPERATION = None
__STATES = Enum("F", "B", "AT", "CW", "CCW")
__SPEED = 0.5 # meters / s
__WHEEL_DIAMETER = 0.33 # meters
__WHEEL_RADIUS = 0.165 # meters
__ANGULAR_SPEED = 0.2
__SLEEP_TIME = 2 # sec

def odometry_callback(msg):
    global __CURRENT_POSE
    __CURRENT_POSE = msg.pose.pose

def drive_forward(distance):
        global __CMD_VEL_PUBLISHER, __SPEED, __SLEEP_TIME
        vel_msg = Twist()
        vel_msg.linear.x = __SPEED
        current_distance = 0
        t0 = rospy.Time.now().to_sec()
        while current_distance <  distance:
            __CMD_VEL_PUBLISHER.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = __SPEED * (t1 - t0)
        vel_msg.linear.x = 0
        __CMD_VEL_PUBLISHER.publish(vel_msg)
        rospy.sleep(__SLEEP_TIME)

def turn(clockwise):
    global __CMD_VEL_PUBLISHER, __ANGULAR_SPEED, __SLEEP_TIME, __CURRENT_POSE
    angle = 1.54
    vel_msg = Twist()
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x=0
    vel_msg.angular.y=0
    euler = euler_from_quaternion((
    __CURRENT_POSE.orientation.x,
    __CURRENT_POSE.orientation.y,
    __CURRENT_POSE.orientation.z,
    __CURRENT_POSE.orientation.w
    ))
    orig_yaw = euler[2]
    yaw = euler[2]
    # angle = yaw * angle

    if clockwise:
        vel_msg.angular.z = -abs(__ANGULAR_SPEED)
    else:
        vel_msg.angular.z = abs(__ANGULAR_SPEED)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    while abs(orig_yaw - yaw) <= angle:
        print str(abs(orig_yaw - yaw)) + " " + str(angle)
        __CMD_VEL_PUBLISHER.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        euler = euler_from_quaternion((
        __CURRENT_POSE.orientation.x,
        __CURRENT_POSE.orientation.y,
        __CURRENT_POSE.orientation.z,
        __CURRENT_POSE.orientation.w
        ))
        yaw = euler[2]

    vel_msg.angular.z = 0
    __CMD_VEL_PUBLISHER.publish(vel_msg)
    rospy.sleep(__SLEEP_TIME)

def list_callback(roi):
    global __CURRENT_POSE, __STATES
    heading = 0
    length = roi.height
    width = roi.width
    row_count = int(width / __ROBOT__WIDTH__)
    start_left = roi.do_rectify
    current_row = 0
    last_state = None
    last_heading = None
    current_state = __STATES.F

    if start_left:
        last_turn = __STATES.CCW
    else:
        last_turn = __STATES.CW

    while (current_row <= row_count):
        if current_state is __STATES.F:
            drive_forward(length - (__ROBOT__TURN_AROUND_LENGTH / 2))
            if last_turn is __STATES.CCW:
                current_state = __STATES.CW
            else:
                current_state = __STATES.CCW
            last_state = __STATES.F
        elif current_state is __STATES.AT:
            drive_forward((__ROBOT__TURN_AROUND_LENGTH / 2) - __ROW_OVERLAP__)
            current_state = last_turn
            last_state = __STATES.AT
        elif current_state is __STATES.CW:
            turn(True)
            last_turn = __STATES.CW
            if last_state is __STATES.AT:
                current_row += 1
                current_state = __STATES.F
            else:
                current_state = __STATES.AT
            last_state = __STATES.CW
        elif current_state is __STATES.CCW:
            turn(False)
            last_turn = __STATES.CCW
            if last_state is __STATES.AT:
                current_row += 1
                current_state = __STATES.F
            else:
                current_state = __STATES.AT
            last_state = __STATES.CCW



def pose_callback(msg):
    global __CURRENT_POSE
    __CURRENT_POSE = msg.pose.pose

if __name__ == '__main__':
    global __CMD_VEL_PUBLISHER
    # setup ROS
    rospy.init_node("husky_drive_grid", anonymous=True)
    # setup listeners
    rospy.Subscriber("weederbot/grid", RegionOfInterest, list_callback)
    rospy.Subscriber("/odometry/filtered", Odometry, odometry_callback)
    __CMD_VEL_PUBLISHER = rospy.Publisher('/platform_control/cmd_vel', Twist)
    rospy.spin()
