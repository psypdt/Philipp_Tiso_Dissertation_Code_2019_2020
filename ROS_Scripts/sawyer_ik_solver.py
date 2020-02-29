#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



from __future__ import print_function

import rospy

import Tkinter as tk
import tkMessageBox

import intera_interface
from intera_interface import CHECK_VERSION

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import ( 
    Header, 
    String,
    Bool
)
 
from sensor_msgs.msg import JointState
from intera_examples.msg import SortableObjectMessage as SortableObjectMsg

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)



class IKSolver:

    def __init__(self):
        self.is_adding_new_item = False  # This flag will be set to true if the user is in close proximity to the robot, flex_ik_service_client should immediatly exit
        self.has_returned_home = False  # False implies "The user wants to create an object, but im not in the default position, so I'll move there now"

        node = rospy.init_node("sawyer_ik_solver_node")
        rate = rospy.Rate(10)  # Publishing rate in Hz
        
        self.arm_speed = 0.28
        self.arm_timeout = 5

        self.ik_sub_abort_sorting = rospy.Subscriber('ui/user/has_aborted', Bool, callback=None, queue_size=10)  # This will suspend all sorting 

        self.ik_sub_add_item = rospy.Subscriber('ui/user/is_moving_arm', Bool, callback=self.disable_sorting_capability_callback, queue_size=10)
        self.ik_pub_current_arm_pose = rospy.Publisher('sawyer_ik_sorting/sawyer_arm/pose/current', Pose, queue_size=10)  # Publish current arm pose

        self.ik_sub_locating_object_done = rospy.Subscriber('ui/new_object/state/is_located', Bool, callback=self.send_current_pose_callback, queue_size=10)  

        #  Starts-up/enables the robot 
        try:
            rs = intera_interface.RobotEnable(CHECK_VERSION)
            init_state = rs.state().enabled
            rs.enable()
        except Exception as ex:
            print(ex)
            error_name = "IK solver crashed!"
            error_msg = "The Inverse Kinematic solver has crashed. Moving the robot is no longer possible.\n\nPlese restart the program."
            self.ik_solver_error_msg(error_name, error_msg)  # Display error if robot can't be enabled
            rospy.signal_shutdown("Failed to enable Robot")

        #  Create a publisher that will publish strings to the ik_status topic
        self.ik_pub_sorted = rospy.Publisher('sawyer_ik_sorting/sortable_objects/object/sorted', SortableObjectMsg, queue_size=10)  # Publish to this topic once object has been sorted
        self.ik_pub_sorting_error = rospy.Publisher('sawyer_ik_solver/sorting/has_failed', SortableObjectMsg, queue_size=10)  # This will publish the object & container pair which has failed

        ik_sub_sorting = rospy.Subscriber('ui/sortable_object/sorting/execute', SortableObjectMsg, callback=self.sort_object_callback, queue_size=4)  # Subscribe to topic where sortable messages arrive
        ik_sub_shutdown = rospy.Subscriber('sawyer_ik_solver/change_to_state/shudown', Bool, callback=self.shutdown_callback, queue_size=10)  # Topic where main_gui tells solver to shutdown

        #  Move to default position when the ik solver is initially launched
        self.sawyer_arm = intera_interface.Limb('right')
        self.sawyer_arm.move_to_neutral(timeout=self.arm_timeout, speed=self.arm_speed) 
        
        rospy.spin()



    #  Create a more flexible IK client 
    #  NOTE This function will retun True if a path was found or False if it failed
    def flex_ik_service_client(self, i_Limb="right", i_Pose=None, i_UseAdvanced=False):
        
        if self.is_adding_new_item == True:  # Can't move if user is in close proximity manually moving the arm
            return False

        if i_Pose == None:
            rospy.logerr("Error: flex_ik_service_client received 'None' in arg 'i_Pose' for target position")
            return False


        #  Initialize IK service
        service_name = "ExternalTools/" + i_Limb + "/PositionKinematicsNode/IKService"
        ik_ServiceClient = rospy.ServiceProxy(service_name, SolvePositionIK)  # Creates a handle to some service that we want to invoke methods on, in this case the SolvePositionIK
        ik_ServiceReq = SolvePositionIKRequest()  # Create a request message that will give us the inverse kinematics for some set of joints (Note that this is done at compile time) 
        msg_header = Header(stamp=rospy.Time.now(), frame_id='base')  # Generates a Header for the message (Think of HTTP)

        #  Set the goal positions for the limb that we specified (I'm pretty sure that it's for the last joint right_j6)
        #  Set the header that we time stamped 
        #  Add Pose object that contains a Point and Quaternion 
        goal_positions = {
            'right': PoseStamped(
                header=msg_header,
                pose=i_Pose
            ),
        }

        ##############################################################################################
        #
        #  SOLVER IS SETUP TO FIND PATH USING SIMPLE INVERSE KINEMATICS
        #
        ##############################################################################################

        #  Add the goal positions for the inverse kinematics
        ik_ServiceReq.pose_stamp.append(goal_positions[i_Limb])

        #  Request the inverse kinematics from the base to the "right_hand" link
        ik_ServiceReq.tip_names.append('right_hand')



        ##############################################################################################
        #
        #  SETUP THE SOLVER TO FIND A PATH USING ADVANCED INVERSE KINEMATICS
        #
        ##############################################################################################

        if (i_UseAdvanced):
            rospy.loginfo("Using Advanced IK Service")

            #  Define the joint seed. If the solver encounters the specified value for a joint, the it will attempt to do some optimisation
            ik_ServiceReq.seed_mode = ik_ServiceReq.SEED_USER
            seed = JointState()  #  JointState describes the state of each joint (possition, velocity, effort)
            seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']  #  The name of the various joints

            seed.position = [0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]  #  The joint angle at which the solver tries to do optimisation for the respective joints

            #  Pass the seeded angles to the Solver via the SolvePositionIKRequest object
            ik_ServiceReq.seed_angles.append(seed)


            #  NOTE Once the Primary IK Task has been solved, the IK Solver will try to bias the joint angles towards the goal joint configuration
            #  NOTE The null space represents the extra degrees of freedom that the joints can move through without affecting the results of the Primary IK Task
            #  Here we try to solve the Primary IK Task
            ik_ServiceReq.use_nullspace_goal.append(True)

            #  NOTE The null space can either be a subset or the full set of joint angles
            goal = JointState()

            #  This is the subset of joints that make up the null space "Joints A, B, C ... can try to move, but can't affect the result"
            goal.name = ['right_j1', 'right_j2', 'right_j3']  
            goal.position = [0.1, -0.3, 0.5]  #  If these possitions are encountered, try to do some optimisation
            ik_ServiceReq.nullspace_goal.append(goal) 

            ik_ServiceReq.nullspace_gain.append(0.4)
        # else:
        #     rospy.loginfo("Using Simple IK Solver")


        ##############################################################################################
        #
        #  PASS THE INVERSE KINEMATICS REQUEST TO THE SOLVER TO GET AN ACTUAL PATH FOR THE ROBOT
        #
        ##############################################################################################
        # rospy.loginfo("Simple IKService Solver Running...")

        try:
            rospy.wait_for_service(service_name, timeout=self.arm_timeout)  #  Waits for the service (5 seconds), creates the service if it doesn't already exist
            response = ik_ServiceClient(ik_ServiceReq)  #  Get the response from the client, contains the joint positions  
        except (rospy.ServiceException, rospy.ROSException), ex:
            rospy.logerr("Service Call Failed: %s" % (ex,))
            return False

        # rospy.loginfo("Advanced IKService Solver Running... ")


        #  Check if the result is valid, and what seed was used to obtain the solution
        if (response.result_type[0] > 0):
            seed_str = {
                        ik_ServiceReq.SEED_USER: 'User Provided Seed',
                        ik_ServiceReq.SEED_CURRENT: 'Current Joint Angles',
                        ik_ServiceReq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(response.result_type[0], 'None')
            
            rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))

            #  Format the joints such that they can be passed to the robot by making a Limb-API complient dictionary
            joint_solution = dict(zip(response.joints[0].name, response.joints[0].position))

            # rospy.loginfo("\nIK Joint Solution:\n%s", joint_solution)
            # rospy.loginfo("------------------")
            # rospy.loginfo("Response Message:\n%s", response)

            # rospy.loginfo("Moving To Target Pose...")

            #  Move the limb into the final position
            self.sawyer_arm = intera_interface.Limb(i_Limb)
            self.sawyer_arm.set_joint_position_speed(self.arm_speed)  # Max ratio of joint speed is 0.2 (pretty slow)
            self.sawyer_arm.move_to_joint_positions(joint_solution)
            rospy.sleep(0.01)
        else:
            rospy.logerr("INVALID POSE - Unable to find POSE")
            return False

        return True
        




    #  This function gets called as soon as we get some data from the "move_to_dest/goal" topic
    #  Data contains both the start and the end possition respectively
    def sort_object_callback(self, data):
        
        if self.is_adding_new_item == True:  # Return immediatly, user is in close proximity, not allowed to make any move 
            return 
        
        #  NOTE For the simulation, have timeout=5 and speed=0.2 otherwise it segfaults, or timeout=2, speed=0.28
        self.sawyer_arm.move_to_neutral(timeout=self.arm_timeout, speed=self.arm_speed)  # The smaller the speed value, the slower the joint movement, note that the movement will stop the moment the timeout is reached
        rospy.sleep(2)

        #  Move to the object which will be picked up
        if self.flex_ik_service_client(i_Pose=data.msg_object_pose, i_UseAdvanced=True):
            rospy.loginfo("Route to object was successfully executed")
            rospy.sleep(2)  # Sleep to simulate object being picked up
        else:
            self.ik_pub_sorting_error.publish(data)  # Send object which has failed
            #  Inform user of failure 
            object_name = data.object_name
            rospy.logwarn("Route Execution Failed: Invalid target object position")
            error_name = "Route Execution Failed (Invalid Position)"
            error_msg = "Object \'%s\' can't be reached\n\nPess 'OK' to continue sorting." % str(object_name)
            
            #  Display error 
            self.ik_solver_error_msg(error_name, error_msg)

            self.sawyer_arm.move_to_neutral(timeout=self.arm_timeout, speed=self.arm_speed)
            rospy.sleep(4)
            return

        self.sawyer_arm.move_to_neutral(timeout=self.arm_timeout, speed=self.arm_speed)

        #  Move the object to the location where the container is
        if self.flex_ik_service_client(i_Pose=data.msg_container_pose, i_UseAdvanced=True):
            rospy.loginfo("Route to container was successfully executed")
            
            self.ik_pub_sorted.publish(data)
            rospy.sleep(2)
        else:
            self.sorting_erik_pub_sorting_errorror_pub.publish(data)
            #  Inform user that container is unreachable
            container_name = data.container_name
            rospy.logwarn("Route Execution Failed: Invalid target container position")
            error_name = "Route Execution Failed (Invalid Position)"
            error_msg = "Container \'%s\' can't be reached\n\nPress 'OK' to continue sorting" % str(container_name)

            #  Display error 
            self.ik_solver_error_msg(error_name, error_msg)

            self.sawyer_arm.move_to_neutral(timeout=self.arm_timeout, speed=self.arm_speed)
            rospy.sleep(4)




    ##  Simple callback that prevents the robot from doing any sorting while human is working with it
    def disable_sorting_capability_callback(self, msg):
        self.is_adding_new_item = msg.data

        #  User wants to create object and arm is not in default position
        if msg.data == True and self.has_returned_home == False: 
            self.sawyer_arm.move_to_neutral(timeout=self.arm_timeout, speed=self.arm_speed)  # Move the arm to default, then disable movement 
            self.has_returned_home = True

        #  Reset the has_returned_home flag once the user is done moving the arm manually
        if msg.data == False and self.has_returned_home == True:
            self.has_returned_home = False  


    ##  This method will send the current arm position to the add_object_pose_node 
    def send_current_pose_callback(self, msg):
        self.is_adding_new_item = msg.data

        end_point = self.sawyer_arm.endpoint_pose()
        
        current_pose = Pose()
        current_pose.position = end_point['position']
        current_pose.orientation = end_point['orientation']

        if self.is_adding_new_item:
            self.ik_pub_current_arm_pose.publish(current_pose)



    def ik_solver_error_msg(self, errorTitle, errorMsg):
        error_title = "Error: " + errorTitle
        root = tk.Tk()
        root.withdraw()
        tkMessageBox.showerror(error_title, errorMsg)
        root.destroy()
    

    def shutdown_callback(self, data):
        if data == True:
            rospy.signal_shutdown("Main GUI terminated")




if __name__ == '__main__':
    IKSolver()