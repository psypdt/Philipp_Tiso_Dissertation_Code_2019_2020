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

#  TODO Find way to import desk into this sim


"""
Intera RSDK Inverse Kinematics Example
"""
from __future__ import print_function

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

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
    Float64MultiArray,
)
 
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


#  Create a more flexible IK client 
#  TODO Look into using arm calibration, could it be useful here?
#  TODO Need to find a way to do collision avoidance  
#  NOTE This function will retun True if a path was found or False if it failed
def flex_ik_service_client(i_Limb="right", i_TargetPose=[0.4950628,-0.0002616,0.3974473], i_TargetOrientation=[0.704020578925,0.710172716916,0.00244101361829,0.00194372088834], i_UseAdvanced=False):
    """ 
    i_Limb
        default: "right"
        -- The robots limb (For Sawyer there is really only one option which is 'right')
    
    i_TargetPose 
        default: [  x= 0.4950628752997,
                    y= -0.000261615832271,
                    z= 0.397447307078] Curren path would probably collide with the table
        -- The cartesian position of the outer most joint [x, y, z] (If the gripper is attatched the target z value will be changed by -0.04804363884)
    
    i_TargetOrientation
        default: [  x=0.704020578925,
                    y=0.710172716916,
                    z=0.00244101361829,
                    w=0.00194372088834]
        -- The target orientation [x, y, z, w]
    
    i_UseAdvanced
        default: true
        -- Will run some optimization after the IK result has been calculated
    """

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
            pose=Pose(
                position=Point(
                    x=i_TargetPose[0],
                    y=i_TargetPose[1],
                    z=i_TargetPose[2],
                ),
                #  A Quaternion descibes the rotation of an object in 3D, we need 4D numbers to represent this (i^2 = j^2 = k^2 = ijk = -1)
                #  if w = 1, then there is no rotation around the x/y/z (roll/pitch/yaw) in radians
                #  Apparently, w is the angle (probably in rads) by which we rotate around an axis (TODO: find out which axis)
                #  x,y,z DONT REPRESENT THE AXIS. Basically all 4 numbers do the same thing, values represent a point on the unit circle in 2D
                #  TODO: Watch Humane Rigging 03 - 3D Bouncy Ball 05 - Quaternion Rotation
                orientation=Quaternion(
                    x=i_TargetOrientation[0],
                    y=i_TargetOrientation[1],
                    z=i_TargetOrientation[2],
                    w=i_TargetOrientation[3],
                ),
            ),
        ),
    }

    ##################################################################################################################################################################################
    #
    #  SOLVER IS SETUP TO FIND PATH USING SIMPLE INVERSE KINEMATICS
    #
    ##################################################################################################################################################################################

    #  Add the goal positions for the inverse kinematics
    ik_ServiceReq.pose_stamp.append(goal_positions[i_Limb])

    #  Request the inverse kinematics from the base to the "right_hand" link
    ik_ServiceReq.tip_names.append('right_hand')



    ##################################################################################################################################################################################
    #
    #  SETUP THE SOLVER TO FIND A PATH USING ADVANCED INVERSE KINEMATICS
    #
    ##################################################################################################################################################################################

    if (i_UseAdvanced):
        rospy.loginfo("Using Advanced IK Service")

        #  TODO Look into what seed actually does
        #  Define the joint seed. If the solver encounters the specified value for a joint, the it will attempt to do some optimisation
        ik_ServiceReq.seed_mode = ik_ServiceReq.SEED_USER
        seed = JointState()  #  JointState describes the state of each joint (possition, velocity, effort)
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']  #  The name of the various joints

        #  TODO Figure out why these values are used
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

        #  TODO Figure out what this means, and why the null space gain must be [0.0, 1.0] or by default 0.4 if its empty
        ik_ServiceReq.nullspace_gain.append(0.4)
    else:
        rospy.loginfo("Using Simple IK Solver")


    ##################################################################################################################################################################################
    #
    #  PASS THE INVERSE KINEMATICS REQUEST TO THE SOLVER TO GET AN ACTUAL PATH FOR THE ROBOT
    #
    ##################################################################################################################################################################################
    rospy.loginfo("Simple IKService Solver Running...")

    try:
        rospy.wait_for_service(service_name, timeout=5.0)  #  Waits for the service (5 seconds), creates the service if it doesn't already exist
        response = ik_ServiceClient(ik_ServiceReq)  #  Get the response from the client, contains the joint positions  
    except (rospy.ServiceException, rospy.ROSException), ex:
        rospy.logerr("Service Call Failed: %s" % (ex,))
        return False

    rospy.loginfo("Advanced IKService Solver Running... ")

    #  TODO Check what result_type contains
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

        rospy.loginfo("\nIK Joint Solution:\n%s", joint_solution)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", response)

        rospy.loginfo("Moving To Target Pose...")
        #  Move the limb into the final position
        sawyer_arm = intera_interface.Limb(i_Limb)
        sawyer_arm.set_joint_position_speed(0.28)  #  Max ratio of joint speed is 0.2 (pretty slow)
        sawyer_arm.move_to_joint_positions(joint_solution)
        rospy.sleep(0.01)
    else:
        rospy.logerr("INVALID POSE - Unable to find POSE")
        return False

    return True
    




#  This function gets called as soon as we get some data from the "destination_coordinate_topic" topic
def test_callback(data):
    # input_position = [0.5, -0.3, 0.1]
    
    print('Got data: {}'.format(data))

    input_position = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    print(input_position)

    # input_orientation = [0, 1, 1, 0]  #  It looks like this is a normalized form of the quaternions
    input_orientation = [0.704020578925,0.710172716916,0.00244101361829,0.00194372088834]
    input_orientation = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

    tmp_arm = intera_interface.Limb('right')
    
    #  NOTE For the simulation, have timeout=5 and speed=0.2 otherwise it segfaults, or timeout=2, speed=0.28
    tmp_arm.move_to_neutral(timeout=15, speed=0.25)  #  The smaller the speed value, the slower the joint movement, note that the movement will stop the moment the timeout is reached
    rospy.sleep(2)

    if flex_ik_service_client(i_TargetPose=input_position,i_TargetOrientation=input_orientation, i_UseAdvanced=True):
        rospy.loginfo("Route was successfully executed")

        rospy.loginfo("Publishing...")
        mesg = "Done"
        pub.publish(mesg)
        rate.sleep()

    else:
        rospy.logwarn("Route Execution Failed: Invalid target position")





def main():
    """RSDK Inverse Kinematics Example
    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.
    Run this example, the example will use the default limb
    and call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    #  Create a publisher that will publish strings to the ik_status topic
    pub = rospy.Publisher('ik_status', String, queue_size=10)  

    rospy.init_node("rsdk_flex_ik_service_client")
    rate = rospy.Rate(10)  #  Publishing rate in Hz

    # Starts-up/enables the robot 
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    subscriber = rospy.Subscriber('move_to_dest/goal', PoseStamped, callback=test_callback)
    rospy.spin()




if __name__ == '__main__':
    main()
