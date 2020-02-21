#!/usr/bin/env python


import rospy
import intera_interface
from intera_interface import (
    CHECK_VERSION,
    Gripper,
    RobotEnable,
)


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import intera_external_devices  #  This is used to get keyboard input

from std_msgs.msg import String



#  This method will close the gripper
def callback_func(data):
    rospy.loginfo("I got the message")

    try:
        grip = intera_interface.Gripper('right_gripper')
    except ValueError:
        rospy.logerr("ERROR: Unable to detect Gripper")
        return
    
    rospy.loginfo("Gripper position is {}".format(grip.get_position()))
    
    tmp_limb = intera_interface.Limb('right')
    end_pt = tmp_limb.endpoint_pose()['position']

    print("Endpoint is x: {x}\n y: {y}\n z: {z}".format(x=end_pt[0], y=end_pt[1], z=end_pt[2]))

    # new_pt = Point(
    #     x = end_pt[0],
    #     y = end_pt[1],
    #     z = end_pt[2]-0.09
    # )

    # tmp_limb.set_joint_trajectory(["right_j6", "right_j4"], new_pt, [0.3, 0.3], [0.01, 0.01])

    grip.set_position(0.01)

    rospy.loginfo("Gripper position is {}".format(grip.get_position()))
    rospy.signal_shutdown("Finished grip action")



def listener():
    rospy.loginfo("Initializing grip_listener Node")
    rospy.init_node('grip_listener', anonymous=True)

    #  Setup the robot state
    rs = intera_interface.RobotEnable(CHECK_VERSION)  #  Robot state
    init_state = rs.state()

    #  Subscribe to the ik_status topic
    rospy.Subscriber("ik_status", String, callback_func)
    
    try:
        grip = intera_interface.Gripper('right_gripper')
    except ValueError:
        rospy.logerr("ERROR: Unable to detect Gripper")
        return

    grip.open()

    rospy.loginfo("grip_listener now waiting for message... ")
    rospy.spin()


if __name__ == "__main__":
    listener()
