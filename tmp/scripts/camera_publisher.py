#!/usr/bin/env python

import numpy as np

import rospy
import intera_interface

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image



#  This method will send the RAW image to some subscribers, they will need to the the cv transform themselves
def pub_image_callback(image_data, image_publisher):    
    
    #  Ensure that we are still up and running 
    if not rospy.is_shutdown():
        image_publisher.publish(image_data)

    # rospy.loginfo("Published an image to 'raw_image_publisher' topic")

    #  Convert the image into one that we can display in openCv, this one is NOT sent to the subscriber
    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(image_data, "bgr8")  #  Creates a openCv compatible image from the data sawyer sent us

    window_name = 'Publisher window'
    cv2.namedWindow(window_name, 0)
    cv2.imshow(window_name, cv_image)
    cv2.waitKey(3)





def main():
    img_publisher = rospy.Publisher('raw_image_publisher', Image, queue_size=10)  #  Create a publisher that gets passed to the callback so that we can push an image to some subscribers
    rospy.init_node('image_pub_node', anonymous=True)

    rate = rospy.Rate(10)  #  Set the publishing rate for the images

    camera = intera_interface.Cameras()
    camera.start_streaming('head_camera')  #  Pick the head camera by default

    #  Create callback function that gets called every time an image is received from the integrated camera
    camera.set_callback(camera_name='head_camera', callback=pub_image_callback, callback_args=img_publisher)  #  NOTE The image data is pass already (otherwise why have the callback) so we dont pass anything in here


    #  This method ensures that we shutdown the node in a way that leaves no 'garbage' behind
    def clean_shutdown():
        rospy.loginfo("Shutting down image_pub_node...")
        cv2.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)

    rospy.spin()



if __name__ == "__main__":
    main()
