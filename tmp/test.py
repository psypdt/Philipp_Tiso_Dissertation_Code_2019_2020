#!/usr/bin/env python


import rospy
import intera_interface
import time


# This function just resets the right joint possition to 0  
def reset():
	limb = intera_interface.Limb('right')
	
	# default_joint_angle = {'right_j0':0.0, 'right_j1':0.0, 'right_j2':0.0, 'right_j4':0.0}
	# limb.move_to_joint_positions(default_joint_angle)
	limb.move_to_neutral(timeout=5, speed=0.4)



# Sets the arm into a know position from which we can work
def start_position():
	limb = intera_interface.Limb('right')  # Create instance of a limb (the arm) to interface with the robot
	gripper = intera_interface.Gripper()  # Create an instance of the gripper __init__(self, side='right', calibrate=True) 

	# Define default position for arm joints, if the joint is already at that position it takes 15sec for it to timeout by default
	default_joint_angle = {
		'right_j0':0.0, 
		'right_j1':0.0, 
		'right_j2':0.0, 
		'right_j3':0.0,
		'right_j4':0.0,
		'right_j5':0.0,
		'right_j6':-1.4
	}
	# right_j6:1.8 is about a 90 degree rotation counter-clockwise, -1.4 is the mirror of 1.8 (use -1.4 as starting val)

	limb.move_to_joint_positions(default_joint_angle, timeout=5.0)  # Move the arm to the appropriate joint angles
	# limb.move_to_neutral(timeout=5, speed=0.3)  # Moves the limbs to a predefined location (consider using this to position the robot when its done working)
	gripper.open()  # Gripper is open by default open(self, position=0.41667)
	


## Some function that will just do some random thing
def doStuff():
	reset()
	#rospy.init_node('Pls_move')  # Initializes a node with name Pls_move
	limb = intera_interface.Limb('right')  # Gets the right arm (not like theres another one)
	
	angles = limb.joint_angles()  # Gets the joint angles
	
	print(angles['right_j0'])
	
	new_right_j0 = angles['right_j0'] + 0.55
	
	joint_dict = {'right_j0':new_right_j0}  # Create dictionary with joint:value mapping
	limb.move_to_joint_positions(joint_dict)  # Move the joints to the dictionary specific positions
	
	new_angles = limb.joint_angles()
	
	print(new_angles['right_j0'])
	
	time.sleep(3.0)
	
	joint_dict['right_j0'] -= 0.55 
	limb.move_to_joint_positions(joint_dict)
	
	print(limb.joint_angles()['right_j0'])



if __name__ == '__main__':
	rospy.init_node('my_node', anonymous=True)
	start_position()