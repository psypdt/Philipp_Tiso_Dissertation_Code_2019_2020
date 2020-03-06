#!/usr/bin/env python

# TODO Make a class that is flexible and allows for easy gripper control
import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import copy
[]


class PickStuff(object):

    def __init__(self):
        rospy.init_node('my_gripper_node')
        self.limb = intera_interface.Limb("right")
        self.gripper = intera_interface.Gripper()

        self.hover_dist = 3.5  #  Safe distance over object


        try:
            rs = intera_interface.RobotEnable(CHECK_VERSION)
            init_state = rs.state().enabled
            rs.enable()
        except Exception as ex:
            print(ex)
            error_name = "IK solver crashed!"
            error_msg = "The Inverse Kinematic solver has crashed. Moving the robot is no longer possible.\n\nPlese restart the program."
        
        self.limb.move_to_neutral(5, 0.2)
        self.gripper.open()
        rospy.sleep(2)

        start = Pose()
        start.position.x  = 0.746485882426
        start.position.y = 0.174683269138
        start.position.z = 0.0501388138564
        start.orientation.x = 0.699006931744
        start.orientation.y = 0.718442492878
        start.orientation.z = -0.03641998505
        start.orientation.w = -0.0103409711763


        # hover = copy.deepcopy(start)
        # hover.position.z = start.position.z + 0.05
        # angles = self.limb.ik_request(hover, "right_gripper_tip")
        # self.limb.move_to_joint_positions(angles)
        
        # self._approach_object(hover, 400)
        self.limb.set_joint_position_speed(0.1)
        self.approach_object(start, 0.01, 'right_gripper_tip')
        self.servo_and_pickup_object(start, steps=400)

    


    
    def approach_object(self, approach_pose, hover_dist, trip_name):
        approach = copy.deepcopy(approach_pose)

        approach.position.z = approach.position.z + hover_dist
        joint_angels = self.limb.ik_request(approach, trip_name)
        self.limb.set_joint_position_speed(0.25)
        
        self.limb.move_to_joint_positions(joint_angels)
        self.limb.set_joint_position_speed(0.1)




    ##  This method will be used to approach an object and grab it
    def servo_and_pickup_object(self, final_pose, time=4.0, steps=400):
        #  Slow the robot down a bit
        self.limb.set_joint_position_speed(0.1)
        print("Setting speed")
        rospy.sleep(1)

        #  Get the current position of the gripper/arm
        current_pose = self.limb.endpoint_pose()
        r = rospy.Rate(1/(time/steps))  # Reset do default publishing rate with a timeout of 4

        print("Getting diff")
        rospy.sleep(1)
        #  Calculate differance between the current position and the desired end position
        ik_delta = Pose()
        ik_delta.position.x = (current_pose['position'].x - final_pose.position.x) / steps 
        ik_delta.position.y = (current_pose['position'].y - final_pose.position.y) / steps
        ik_delta.position.z = (current_pose['position'].z - final_pose.position.z) / steps
        ik_delta.orientation.x = (current_pose['orientation'].x - final_pose.orientation.x) / steps
        ik_delta.orientation.y = (current_pose['orientation'].y - final_pose.orientation.y) / steps
        ik_delta.orientation.z = (current_pose['orientation'].z - final_pose.orientation.z) / steps
        ik_delta.orientation.w = (current_pose['orientation'].w - final_pose.orientation.w) / steps

        print("Got diff: %s" % ik_delta)
        rospy.sleep(1)
        #  Move towards the object from the current position
        for d in range(int(steps), -1, -1):
            ik_step =Pose()

            ik_step.position.x = d*ik_delta.position.x + final_pose.position.x
            ik_step.position.y = d*ik_delta.position.y + final_pose.position.y
            ik_step.position.z = d*ik_delta.position.z + final_pose.position.z
            ik_step.orientation.x = d*ik_delta.orientation.x + final_pose.orientation.x
            ik_step.orientation.y = d*ik_delta.orientation.y + final_pose.orientation.y
            ik_step.orientation.z = d*ik_delta.orientation.z + final_pose.orientation.z
            ik_step.orientation.w = d*ik_delta.orientation.w + final_pose.orientation.w
            joint_angles = self.limb.ik_request(ik_step, 'right_gripper_tip')

            if joint_angles:
                print("Approaching now")
                self.limb.move_to_joint_positions(joint_angles)
            r.sleep()

        rospy.sleep(0.4)
        self._close_gripper(0.0075)  # Grab object
        rospy.sleep(0.5)

        
        
    ##  Closes the gripper to some degree
    def _close_gripper(self, degree=0):
        self.gripper.close(degree)
        rospy.sleep(0.01)





    ##  Opens the gripper
    def _open_gripper(self):
        self.gripper.open()
        rospy.sleep(0.01)






#  ik_delta = Pose()
# ik_delta.position.x = (final_pose.position.x - current_pose['position'].x) / steps 
# ik_delta.position.y = (final_pose.position.y - current_pose['position'].y) / steps
# ik_delta.position.z = (final_pose.position.z - current_pose['position'].z) / steps
# ik_delta.orientation.x = (final_pose.orientation.x - current_pose['orientation'].x) / steps
# ik_delta.orientation.y = (final_pose.orientation.y- current_pose['orientation'].y) / steps
# ik_delta.orientation.z = (final_pose.orientation.z - current_pose['orientation'].z) / steps
# ik_delta.orientation.w = (final_pose.orientation.w - current_pose['orientation'].w) / steps


if __name__ == "__main__":
    PickStuff()