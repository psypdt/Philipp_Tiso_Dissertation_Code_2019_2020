#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped


## NOTE: This should be integrated into the GUI, that way the GUI decides what gets published, doesn't make sense to split this into 2 nodes (GUI->Position_Publisher)

def main():
    rospy.init_node("position_publisher_node")

    publisher = rospy.Publisher("move_to_dest/goal", PoseStamped, queue_size=10)

    #  Create the PoseStamped object that will contain the target position and orientation
    final_pos = PoseStamped()

    #  Create header for message
    final_pos.header.seq = 1
    final_pos.header.stamp = rospy.Time.now()
    final_pos.header.frame_id = "map"

    #  Final Positions
    final_pos.pose.position.x = 0.704020578925
    final_pos.pose.position.y = 0.6890
    final_pos.pose.position.z = 0.455

    #  Final Orientation
    final_pos.pose.orientation.x = 0.0
    final_pos.pose.orientation.y = 0.0
    final_pos.pose.orientation.z = 0.0
    final_pos.pose.orientation.w = 1.0

    rospy.sleep(1)  # Timeout thread to make sure that subscriber has chance to subscribe to the topic
    publisher.publish(final_pos)

    print("Sent thing")


if __name__ == "__main__":
    main()    
