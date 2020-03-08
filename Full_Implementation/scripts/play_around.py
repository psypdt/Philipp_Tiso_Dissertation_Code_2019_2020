#!/usr/bin/env python


import argparse
import numpy as np

import time
import threading

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy 
import intera_interface
from sensor_msgs.msg import Image
import geometry_msgs.msg
import moveit_commander

	
#  Test if the external webcam is accessible
def test():
	rospy.init_node('my_webcam_node')
	cap = cv2.VideoCapture(0)

	while True:
		ret, cv_image = cap.read()

		gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  #  Grayscales the cv image we obtained from the webcam
		blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 0)  #  Blurs the image

		get_edge = cv2.Canny(blurred_image, 10, 100)  #  Applies the canny edge filter to the blurred image
		final_image = np.hstack([get_edge])

		cv_window_name = "Webcam_Window"
		cv2.namedWindow(cv_window_name, 0)
		cv2.imshow(cv_window_name, final_image)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	
	cap.release()
	cv2.destroyAllWindows()
	rospy.signal_shutdown()

# This function just resets the right joint possition to 0  
def reset():
	limb = intera_interface.Limb('right')
	
	default_joint_angle = {'right_j0':0.0}
	limb.move_to_joint_positions(default_joint_angle)
	
	limb.move_to_neutral()



## Some function that will just do some random thing
def doStuff():
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
	
	
def plsPub(image):
	publsher.publish(image)

	
# This callback function will display the image from the camera (ros camera for now)
def show_image_callback(image_data, (edge_detection, window_name)):
	bridge = CvBridge()
		
	try:
		cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")  # CV image is the input image 
	except:
		rospy.logerr(err)
		return
	
	if edge_detection == True:
		gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) # Grayscale the input image
		blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 0) # Blur the image
		
		get_edge = cv2.Canny(blurred_image, 100, 200)
		cv_image = np.hstack([get_edge]) # Add the image to a stack (im guessing)
		# plsPub(cv_image)
		
	edge_str = "(Edge Detection)" if edge_detection == True else ''
	
	# Create a window that will display the image
	cv_window_name = ' '.join([window_name, edge_str])
	cv2.namedWindow(cv_window_name, 0)
	cv2.imshow(cv_window_name, cv_image) # This will keep refreshing the image on screen, so closing the window won't do much
	cv2.waitKey(3)
	
	
	
# This function will setup the camera and start capturing images from it	
def main():
	rp = intera_interface.RobotParams()
	valid_cameras = rp.get_camera_names()
	
	for c in valid_cameras:
		print('Try: {}'.format(c))
	
	if not valid_cameras:
		rp.log_message(("Unable to detect cameras on robot. Shutting down"), "ERROR")	
		return
	
	arg_fmt = argparse.RawDescriptionHelpFormatter  # Create a formatter/ parser for the user input
	parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
	
	# Add flag for user to say if edge detection should be run or not
	parser.add_argument('-c', '--camera', type=str, default='head_camera', choices=valid_cameras, help='The default camera name for the camera display')
	parser.add_argument('-r', '--raw', action='store_true', help='Should the raw image be used?')
	parser.add_argument('-e', '--edge', action='store_true', help='Should Canny edge detection be streamed? takes argument of type bool.')
	args = parser.parse_args()
	
	# Initialize a new node if the things above have been completed
	print('Initializing new node...')
	rospy.init_node('camera_display', anonymous=True)  # Initializes a new node
	
	
	camera = intera_interface.Cameras()
	
	if not camera.verify_camera_exists(args.camera):
		rospy.logerr("Invalid camera name. Shutting down.")
		return
	
	# web_cam = cv2.VideoCapture(0)
	camera.start_streaming(args.camera)  # Get input form the selected camera
	rectify_image = not args.raw
	use_canny_edge = args.edge
	
	camera.set_callback(args.camera, show_image_callback, rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))  # Send args to callback function 
	
	# Function that performs a clean shutdown
	def clean_shutdown():
		print("Shutting down camera_display node")
		cv2.destroyAllWindows()
	
	rospy.on_shutdown(clean_shutdown)  # Call the clean_shutdown() function when rospy shuts off
	rospy.loginfo("camera_display node is running. Use ctrl+c to quit")
	rospy.spin()  # Keep spinning, "hey python, you can't exit until this node has been shutdown"
	

if __name__ == '__main__':
	# publisher = rospy.Publisher('my_images', Image)
	rospy.init_node('my_node', anonymous=True)
	# main()
	reset()

	
	
	
	
	
	

