#!/usr/bin/env python

# Husky Drive Grid
# By Ryan Owens
# Subscribes to an 'area' measurement and sends move_base Actions to
# cover the area

__ROBOT__WIDTH__ = 0.67 # Meters
__ROBOT__LENGTH__ = 0.99 # Meters
__ROW_OVERLAP__ = 0.05 # Meters
__ROBOT__BASE_LINK__ = "base_link"
__ROBOT__TURN_AROUND_LENGTH = 2 # Meters
__MOVE_BASE_CLIENT = None

import rospy
import actionlib
from sensor_msgs.msg import RegionOfInterest
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# This is all because we don't have enough room for quality landmarks
# active landmarks would be best, but why not just hardcode it instead?
def generate_waypoit_list(length, width, startLeft):
        goals = []
        # determine the number of 'rows'
        row_count = int(width / __ROBOT__WIDTH__)
        # determine the first turn direction
        clockwise_turn = (startLeft == True)
        # for each of the 'rows'
        for _ in range(0, row_count):
            # Drive Down the 'row'
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = length - (__ROBOT__TURN_AROUND_LENGTH / 2)
            goal.target_pose.pose.orientation.w = 1
            goal.target_pose.pose.position.y = 0
            goal.target_pose.pose.position.z = 0
            goals.append(goal)
            # Turn 90`
            turn_goal = MoveBaseGoal()
            if clockwise_turn:
                q = quaternion_from_euler(0, 0, -1.5707) # radians
            else:
                q = quaternion_from_euler(0, 0, 1.5707) # radians
            turn_goal.target_pose.pose.orientation.x = q[0]
            turn_goal.target_pose.pose.orientation.y = q[1]
            turn_goal.target_pose.pose.orientation.z = q[2]
            turn_goal.target_pose.pose.orientation.w = q[3]

            turn_goal.target_pose.pose.position.x = 0
            turn_goal.target_pose.pose.position.y = 0
            turn_goal.target_pose.pose.position.z = 0
            # Add the Turn Goal
            goals.append(turn_goal)
            # Add Straight drive before next turn
            goal = MoveBaseGoal()
            goal.target_pose.pose.orientation.w = 1
            goal.target_pose.pose.position.x = (__ROBOT__TURN_AROUND_LENGTH / 2) - __ROW_OVERLAP__
            goal.target_pose.pose.position.y = 0
            goal.target_pose.pose.position.z = 0
            goals.append(goal)
            # Add another Turn to orient for next 'row'
            turn_goal.target_pose.header.stamp = rospy.get_rostime()
            goals.append(turn_goal)
            # Flip turn direction for next row
            clockwise_turn = not clockwise_turn
        # return the list of goals
        return goals


def callback(roi):
    # ROI has x_offset, y_offset, height, width, do_rectify
    # I am using width as width, height as length, and do_rectify as startLeft
    # Offsets are currently not being used
    waypoints = generate_waypoit_list(roi.height, roi.width, roi.do_rectify)
    print "Received ROI: "
    print roi
    for goal in waypoints:
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.header.frame_id = __ROBOT__BASE_LINK__
        print goal
        __MOVE_BASE_CLIENT.send_goal(goal)
        __MOVE_BASE_CLIENT.wait_for_result()


if __name__ == '__main__':
    # setup ROS
    rospy.init_node("husky_drive_grid", anonymous=True)
    # setup move base client
    __MOVE_BASE_CLIENT = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    __MOVE_BASE_CLIENT.wait_for_server(rospy.Duration(5))
    # setup listeners
    rospy.Subscriber("weederbot/grid", RegionOfInterest, callback)
    rospy.spin()
