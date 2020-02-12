#!/usr/bin/env python

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy 
import intera_interface

def nothing(x):
	pass


def obj_detection_test():
	cap = cv2.VideoCapture(0)

	# cv2.namedWindow("Object detection window")
	# cv2.createTrackbar("LH", "Tracking", 0, 255, nothing)
	# cv2.createTrackbar("LS", "Tracking", 0, 255, nothing)
	# cv2.createTrackbar("LV", "Tracking", 0, 255, nothing)

	# cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
	# cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
	# cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)c

	while True:
		_, image = cap.read()

		#  Construct an hsv image that can be used to identify some blob
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		# l_h = cv2.getTrackbarPos("LH", "Tracking")
		# l_s = cv2.getTrackbarPos("LS", "Tracking")
		# l_v = cv2.getTrackbarPos("LV", "Tracking")

		# u_h = cv2.getTrackbarPos("UH", "Tracking")
		# u_s = cv2.getTrackbarPos("US", "Tracking")
		# u_v = cv2.getTrackbarPos("UV", "Tracking")

		#  l_b = [90, 50, 50] & u_b = [130, 255, 255] will detect BLUE objects

		l_b = np.array([90, 50, 50])  # Lower bound
		u_b = np.array([130, 255, 255])  # Upper bound

		mask = cv2.inRange(hsv, l_b, u_b)

		result = cv2.bitwise_and(image, image, mask=mask)

		cv2.imshow("frame", image)
		cv2.imshow("mask", mask)
		cv2.imshow("result", result)

		if cv2.waitKey(1) == ord('q'):
			break


def olf_func():
	cap = cv2.VideoCapture(0)


	while True:
		ret, cv_image = cap.read()

		gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  #  Grayscales the cv image we obtained from the webcam
		blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 0)  #  Blurs the image

		get_edge = cv2.Canny(blurred_image, 10, 100)  #  Applies the canny edge filter to the blurred image
		final_image = np.hstack([get_edge])

		cv_window_name = "Webcam_Window"
		# cv2.namedWindow(cv_window_name, 0)
		cv2.imshow(cv_window_name, cv_image)

		if cv2.waitKey(1) == ord('q'):
			break





	
#  Test if the external webcam is accessible
def test():
	rospy.init_node('my_webcam_node')
	
	obj_detection_test()
	
	cap.release()
	cv2.destroyAllWindows()
	rospy.signal_shutdown()




if __name__ == "__main__":
    test()