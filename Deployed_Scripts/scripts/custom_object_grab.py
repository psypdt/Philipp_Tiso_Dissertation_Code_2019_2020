#!/usr/bin/env python

import rospy
import intera_interface
from waypoints_class import Waypoints




class CustomObjectGrasping(object):

    def __init__(self):
        
        self.waypoints = Waypoints()

        self.rs = intera_interface.RobotEnable()
        self._init_state = self.rs.state().enabled
        self.rs.enable()

        #  Set up arm, cuff and gripper
        self.sawyer_arm = intera_interface.Limb('right')
        self.sawyer_gripper = intera_interface.Gripper()
        self.sawyer_cuff = intera_interface.Cuff(limb='right')
        self.sawyer_lights = intera_interface.Lights()
        
        #  Move to default position when the ik solver is initially launched
        self.sawyer_arm.move_to_neutral(timeout=10, speed=0.28) 
        self.sawyer_gripper.open()

        self.gripper_dist = self.sawyer_gripper.get_position()

        self.sawyer_cuff.register_callback(self.cuff_open_gripper_callback, '{0}_button_upper'.format('right'))
        self.sawyer_cuff.register_callback(self.cuff_close_gripper_callback, '{0}_button_lower'.format('right'))

        self.set_light_status('red', False)
        self.set_light_status('green', True)
        self.set_light_status('blue', False)



    def do_task(self):
        self.waypoints.record()
        self.waypoints.playback()




    ##  This method will be used to approach an object and grab it
    def servo_and_pickup_object(self, final_pose, time=4.0, steps=600):
        #  Slow the robot down a bit
        self.sawyer_arm.set_joint_position_speed(0.12)

        rospy.sleep(1)

        #  Calculate differance between the current position and the desired end position
        angles = self.sawyer_arm.ik_request(final_pose)
        self.sawyer_arm.move_to_joint_positions(angles) 

        rospy.sleep(1)




    ##  Moves the arm away from the object in a linear fation, it will move back to the hover pose
    def retract_from_object(self, current_pose):
        #  Get joint angles for hover pose
        return_pose = current_pose
        return_pose.position.z = return_pose.position.z + self.hover_dist

        return_angles = self.sawyer_arm.ik_request(return_pose)
        self.sawyer_arm.move_to_joint_positions(return_angles)



    ##  Closes the gripper to some degree
    def _close_gripper(self, degree=0):
        self.sawyer_gripper.close(degree)
        self.sawyer_gripper.set_holding_force(3.0)
        rospy.sleep(0.01)



    ##  Opens the gripper
    def _open_gripper(self):
        self.sawyer_gripper.open()
        rospy.sleep(0.01)




    ##  Callback invoked when user uses cuff to open with gripper
    def cuff_open_gripper_callback(self, value):
        if value and self.sawyer_gripper.is_ready():
            self.sawyer_gripper.open()



    ## Callback invoked when user uses cuff to close gripper
    def cuff_close_gripper_callback(self, value):
        if value and self.sawyer_gripper.is_ready():
            current_position = self.sawyer_gripper.get_position()
            if current_position != 0:
                self.sawyer_gripper.close(current_position-0.01)
                self.gripper_dist = self.sawyer_gripper.get_position()




    ##  Helper method that sets sawyer lights 
    def set_light_status(self, color, status):
        self.sawyer_lights.set_light_state('head_{0}_light'.format(color), on=bool(status))
        self.sawyer_lights.set_light_state('{0}_hand_{1}_light'.format('right', color), on=bool(status))



if __name__ == '__main__':
    rospy.init_node('TestWaypoint')

    custom = CustomObjectGrasping()
    custom.do_task()


    