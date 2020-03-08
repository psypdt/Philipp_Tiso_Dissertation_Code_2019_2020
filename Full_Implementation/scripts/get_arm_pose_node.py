#! /usr/bin/env python



import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose




def publish_arm_pose_callback(data, pub):
    if data:
        arm = intera_interface.Limb('right')
        pose = arm.endpoint_pose()

        pub.publish(pose)





def main():
    pub = rospy.Publisher('robot/arm/right/get_pose', Pose, queue_size=10)  # Publish when user says "An object/container is here"
    sub = rospy.Subscriber('data/request/arm_pose', Bool, callback=publish_arm_pose_callback, callback_args=pub)

    node = rospy.init_node('get_arm_pose_node')

    try:
        robot = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = robot.state().enable
    except Exception as exception:
        print(exception)
        rospy.signal_shutdown("Failed to get robot state")

    rospy.spin()



if __name__ == "__main__":
    main()