#!/usr/bin/env python

from __future__ import print_function
import rospy
import simplejson
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sorting_object_class import SortableObject
from intera_examples.msg import SortableObject as SortableObjectMsg


## NOTE: This should be integrated into the GUI, that way the GUI decides what gets published, doesn't make sense to split this into 2 nodes (GUI->Position_Publisher)

def main():
    rospy.init_node("position_publisher_node")

    publisher = rospy.Publisher("move_to_dest/goal", SortableObjectMsg, queue_size=10)
    rate = rospy.Rate(10)  #  Publishing rate in Hz

    start = Pose()
    # start.header.seq = 1
    # start.header.stamp = 1
    # start.header.frame_id = 'start'

    #  Final Positions
    start.position.x = 0.704020578925
    start.position.y = 0.6890
    start.position.z = 0.455

    #  Final Orientation
    start.orientation.x = 0.0
    start.orientation.y = 0.0
    start.orientation.z = 0.0
    start.orientation.w = 1.0

    #  Create the PoseStamped object that will contain the target position and orientation
    final_pos = Pose()

    #  Create header for message
    # final_pos.header.seq = 1
    # final_pos.header.stamp = rospy.Time.now()
    # final_pos.header.frame_id = 'end'

    #  Final Positions
    final_pos.position.x = 0.704020578925
    final_pos.position.y = 0.6890
    final_pos.position.z = 0.455

    #  Final Orientation
    final_pos.orientation.x = 0.0
    final_pos.orientation.y = 0.0
    final_pos.orientation.z = 0.0
    final_pos.orientation.w = 1.0

    rospy.sleep(1)  # Timeout thread to make sure that subscriber has chance to subscribe to the topic


    send = SortableObjectMsg(name='my thing', start_pose=start, end_pose=final_pos)
    publisher.publish(send)



if __name__ == "__main__":
    main()    
