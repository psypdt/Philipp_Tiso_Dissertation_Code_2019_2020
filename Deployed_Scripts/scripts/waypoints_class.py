#!/usr/bin/env python


import rospy
import intera_interface
import copy


class Waypoints(object):

    def __init__(self, speed=0.28, accuracy=0.003726646, timeout=5):
        self.sawyer_arm = intera_interface.Limb('right')
        self.joint_speed = speed
        self.accuracy=accuracy
        self.timeout = timeout

        self.waypoints = list()
        self._is_recording_waypoints = False

        self.rs = intera_interface.RobotEnable()
        self._init_state = self.rs.state().enabled
        self.rs.enable()

        # Create Navigator
        self.navigator_io = intera_interface.Navigator()
        self.sawyer_lights = intera_interface.Lights()

        # Create Gripper & Cuff
        self.sawyer_gripper = intera_interface.Gripper()
        self.sawyer_cuff = intera_interface.Cuff(limb='right')

        self.sawyer_cuff.register_callback(self.cuff_open_gripper_callback, '{0}_button_upper'.format('right'))
        self.sawyer_cuff.register_callback(self.cuff_close_gripper_callback, '{0}_button_lower'.format('right'))


        # print(self.navigator_io.list_all_items())
        self.sawyer_arm.move_to_neutral(timeout=5, speed=self.joint_speed) 
        self.sawyer_gripper.open()


        self.set_light_status('red', False)
        self.set_light_status('green', False)
        self.set_light_status('blue', True)



    ##  Record the users motions
    def record_waypoint(self, value):
        if value != 'OFF':
            self.waypoints.append((self.sawyer_arm.joint_angles(), self.sawyer_gripper.get_position()))



    ##  Discard all current points
    def discard_waypoints(self, value):
        if value != 'OFF':
            self.waypoints = list()



    ##  Stop recording waypoints
    def stop_recording(self, value):
        if value != 'OFF':
            self._is_recording_waypoints = False
            
            self.set_light_status('red', True)
            self.set_light_status('green', False)
            self.set_light_status('blue', False)

            rospy.sleep(3)
            self.sawyer_arm.move_to_neutral(timeout=5, speed=0.25)
            self.sawyer_gripper.open()


            self.set_light_status('red', False)
            self.set_light_status('green', False)
            self.set_light_status('blue', True)



    ##  Record a sequence of waypoit
    def record(self):
        self.set_light_status('red', False)
        self.set_light_status('green', True)
        self.set_light_status('blue', False)

        # Connect navigator to allow the use of the buttons
        ok_id = self.navigator_io.register_callback(self.record_waypoint, 'right_button_ok')

        # Set the Rethink button to stop the trajectory recording
        show_id = self.navigator_io.register_callback(self.stop_recording, 'right_button_show')

        # Press the circle to reset
        reset_id = self.navigator_io.register_callback(self.discard_waypoints, 'right_button_circle')

        self._is_recording_waypoints = True

        # Wait for user to finish recording waypoints
        while not rospy.is_shutdown() and self._is_recording_waypoints:
            rospy.sleep(1.0)


        # Disconnect callbacks since user has finished
        self.navigator_io.deregister_callback(ok_id)
        self.navigator_io.deregister_callback(show_id)
        self.navigator_io.deregister_callback(reset_id)



    ##  Playback the recorded waypoints
    def playback(self):
        self.set_light_status('red', True)
        self.set_light_status('green', False)
        self.set_light_status('blue', False)
        self.sawyer_arm.set_joint_position_speed(self.joint_speed)
        # print(self.waypoints)
        rospy.sleep(3)
        
        for (waypoint, pos) in self.waypoints:
            self.sawyer_arm.move_to_joint_positions(waypoint, timeout=2, threshold=self.accuracy)
            self.sawyer_gripper.close(pos)

        self.sawyer_arm.move_to_neutral(timeout=5, speed=0.28)  # Move back to dfault position
        self.sawyer_gripper.open()

        #  Signal that robot is in standby
        self.set_light_status('red', False)
        self.set_light_status('green', False)
        self.set_light_status('blue', True)
        self.sawyer_arm.set_joint_position_speed(self.joint_speed)
    


    ##  Closes the gripper to some degree
    def _close_gripper(self, degree=0):
        self.sawyer_gripper.close(degree)
        # self.sawyer_gripper.set_holding_force(3.0)
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

    waypoints = Waypoints()

    waypoints.record()
    waypoints.playback()