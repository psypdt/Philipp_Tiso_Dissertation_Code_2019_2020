#!/usr/bin/env python

# TODO Make a class that is flexible and allows for easy gripper control
import rospy
import intera_interface
from intera_interface import CHECK_VERSION
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import copy



class PickStuff(object):

    def __init__(self):
        self.drop_item_sub = rospy.Subscriber('sawyer_gripper/pickup_item', Bool, callback=self.do_action, queue_size=10)
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
        start.position.x  = 0.739750085115
        start.position.y = 0.123238301586
        start.position.z = 0.0460040844242
        start.orientation.x = 0.699006931744
        start.orientation.y = 0.714007343136
        start.orientation.z = -0.0364803194911
        start.orientation.w = 0.0158748106206


        hover = copy.deepcopy(start)
        hover.position.z = start.position.z + 0.5
        angles = self.limb.ik_request(hover, "right_gripper_tip")
        self.limb.move_to_joint_positions(angles)
        
        # self._approach_object(hover, 400)
        self.limb.set_joint_position_speed(0.1)
        self._approach_object(start, 600)

    



    
    def _approach_object(self, pose, steps):
        current_pose = self.limb.endpoint_pose()
        r = rospy.Rate(1/(4.0/steps)) 

        ik_delta = Pose()
        ik_delta.position.x = (current_pose['position'].x - pose.position.x) / steps 
        ik_delta.position.y = (current_pose['position'].y - pose.position.y) / steps
        ik_delta.position.z = (current_pose['position'].z - pose.position.z) / steps
        ik_delta.orientation.x = (current_pose['orientation'].x - pose.orientation.x) / steps
        ik_delta.orientation.y = (current_pose['orientation'].y - pose.orientation.y) / steps
        ik_delta.orientation.z = (current_pose['orientation'].z - pose.orientation.z) / steps
        ik_delta.orientation.w = (current_pose['orientation'].w - pose.orientation.w) / steps

        rospy.sleep(3)

        for d in range(int(steps), -1, -1):
            ik_step =Pose()

            ik_step.position.x = d*ik_delta.position.x + pose.position.x
            ik_step.position.y = d*ik_delta.position.y + pose.position.y
            ik_step.position.z = d*ik_delta.position.z + pose.position.z
            ik_step.orientation.x = d*ik_delta.orientation.x + pose.orientation.x
            ik_step.orientation.y = d*ik_delta.orientation.y + pose.orientation.y
            ik_step.orientation.z = d*ik_delta.orientation.z + pose.orientation.z
            ik_step.orientation.w = d*ik_delta.orientation.w + pose.orientation.w
            joint_angles = self.limb.ik_request(ik_step, 'right_gripper_tip')

            if joint_angles:
                self.limb.move_to_joint_positions(joint_angles)
            r.sleep()


    def do_action(self, status):
        #  Pickup item
        if status.data == True:
            self.gripper.close(position=0.0075)
            rospy.sleep(2)

        #  Drop item
        else:
            self.gripper.open()
            rospy.sleep(2)



    def approach_object(self, pose):
        approach = copy.deepcopy(pose)

        #  Hover over the object
        # approach.position.z = approach.position.z + self.hover_dist

        # joint_angles = self.limb.ik_request(approach, "pickup_obj")

        
        






if __name__ == "__main__":
    PickStuff()