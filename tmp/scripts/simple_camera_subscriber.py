#!/usr/bin/env python

import numpy as np

import rospy
import intera_interface

import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt

from sensor_msgs.msg import Image, CameraInfo



# class MyImageSubscriber():

#     # Constructor
#     def __init__(self):
#         self.node_name = 'my_image_sub'
#         rospy.init_node(self.node_name)
        
#         # What do we want to do when we shutdown, call the clean_up function
#         rospy.on_shutdown(self.clean_up)

#         self.cv_window_name = "Image window"
        
#         # Create a CvBridge object to translate the ROS image into a CV image
#         self.bridge = CvBridge()

#         # Create the subscriber for the image
#         self.sub = rospy.Subscriber("image_Publisher", Image, callback=self.my_callback)

#         rospy.loginfo("Waiting for image topics... ")


#     # Call back function that will convert the ros image into a cv image 
#     def my_callback(self, ros_img):
#         # Use cv_bridge() to convert the ROS image to OpenCV format
#         try:
#             frame = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
#         except CvBridgeError, e:
#             print e

#         rospy.loginfo("Got an image...")
#         # Create a cv2 window for the image
#         cv2.namedWindow(self.cv_window_name, 0)

#         cv2.imshow(self.node_name, frame)

#         # Convert a frame into a cv image by passing it through a numpy array
#         #frame = np.array(frame, dtype=np.uint8)

#         # process the frame that we got
#         #final_image = np.hstack([self.process_image(frame)])

#         # Display the image in the window we created before 
#         #cv2.imshow(self.node_name, final_image)

#         # Process any keyboard commands
#         self.keystroke = cv.WaitKey(5)
#         if 32 <= self.keystroke and self.keystroke < 128:
#             cc = chr(self.keystroke).lower()
#             if cc == 'q':
#                 # The user has press the q key, so exit
#                 rospy.signal_shutdown("User hit q key to quit.")


#     # This method will perform some operations on the frame that we pass to it, to get an edge detection
#     def process_image(self, frame):
#         # Convert to greyscale
#         grey = cv2.cvtColor(frame, cv2.CV_BGR2GRAY)

#         # Blur the image
#         grey = cv2.blur(grey, (7, 7))

#         # Compute edges using the Canny edge filter
#         edges = cv2.Canny(grey, 15.0, 30.0)

#         return edges

#     # What should happen before the software shuts down
#     def clean_up(self):
#         cv2.destroyAllWindows()


#  This callback method will receive the RAW image data from the camera_publisher
def subscriber_callback(image_data):
    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")  #  Create image from raw data that we received

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  #  Transorm image from BGR to GRAY
    blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)  #  Blur the image using the Gaussian filter, (3,3) represents the size of the 'kernel' or simply the grid

    edge_list = cv2.Canny(blurred_image, 10, 100)  #  Extract the edges (as a list data struct) from the image using the Canny filter

    final_image = np.hstack([edge_list])  #  TODO Find out what this actually does, it would be reasonable that all the detected edges are extracted from the edge_list and are overlayed


    #  Display our results
    window_name = "subscriber_window"
    cv2.namedWindow(window_name, 0)
    cv2.imshow(window_name, final_image)
    cv2.waitKey(3)

    #  Alternative way of showing the image
    # plt.imshow(final_image)
    # plt.show()


#  The listening is happening inside the main function
def main():
    rospy.init_node('raw_image_subscriber_node', anonymous=True)  #  We don't care how many subscribers there are, hence it can be anonymous

    img_subscriber = rospy.Subscriber('raw_image_publisher', Image, subscriber_callback)  #  Subscribes to 'raw_image_publisher', gets back Image, and calls the relevant callback function


    def clean_shutdown():
        rospy.loginfo("Shutting down raw_image_subscriber_node...")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()

