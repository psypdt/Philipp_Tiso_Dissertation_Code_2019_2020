#!/usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


def plan_and_run():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('my_node', anonymous=True)


    group = moveit_commander.MoveGroupCommander("right_arm")

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.8
    pose_goal.position.z = 0.9

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)


if __name__ == "__main__":
    plan_and_run()