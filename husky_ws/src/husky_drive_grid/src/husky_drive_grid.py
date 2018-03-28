#!/bin/env python3

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
# active landmarks would be best, but why not just hardcode it instead? ¯\_(ツ)_/¯
def generate_waypoit_list(length, width, startLeft):
        goals = []
        # determine the number of 'rows'
        row_count = width / __ROBOT__WIDTH__
        # determine the first turn direction
        clockwise_turn = (startLeft == True)
        # for each of the 'rows'
        for _ in range(0, row_count):
            # Drive Down the 'row'
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = length - (__ROBOT__TURN_AROUND_LENGTH / 2)
            goal.target_pose.orientation.w = 1
            goals.append(goal)
            # Turn 90`
            turn_goal = MoveBaseGoal()
            turn_goal.target_pose.header.frame_id = __ROBOT__BASE_LINK__
            turn_goal.target_pose.header.stamp = rospy.now()
            if clockwise_turn:
                q = quaternion_from_euler(0, 0, 1.5707) # radians
            else:
                q = quaternion_from_euler(0, 0, -1.5707) # radians
            turn_goal.target_pose.orientation = q
            # Add the Turn Goal
            goals.append(turn_goal)
            # Add Straight drive before next turn
            goal = MoveBaseGoal()
            goal.target_pose.orientation.w = 1
            goal.target_pose.position.x = (__ROBOT__TURN_AROUND_LENGTH / 2) - __ROW_OVERLAP__
            goals.append(goal)
            # Add another Turn to orient for next 'row'
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
    for goal in waypoints:
        __MOVE_BASE_CLIENT.sendGoal(goal)
        __MOVE_BASE_CLIENT.waitForResult()


if __name__ == '__main__':
    # setup ROS
    rospy.init_node("husky_drive_grid", anonymous=True)
    # setup move base client
    __MOVE_BASE_CLIENT = actionlib.SimpleActionClient("move_base", MoveBaseGoal)
    __MOVE_BASE_CLIENT.wait_for_server(rospy.Duration(5))
    # setup listeners
    rospy.Subscriber("weederbot/grid", RegionOfInterest, callback)
    rospy.spin()
