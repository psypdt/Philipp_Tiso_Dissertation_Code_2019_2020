#! /usr/bin/env python

import rospy
import cv2

import pcl_ros
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as plc2

try:
    import tensorflow 
except ImportError:
    print("Tensorflow cant be found?")

import numpy as np
import openni_camera


#  Callback for the point cloud data
def plc_callback(m_point_cloud):
    assert isinstance(m_point_cloud, PointCloud2)  #  Create instance of PointCloud2 obj with properties and name m_point_cloud
    
    generate = plc2.read_points(m_point_cloud, field_names=('x', 'y', 'z'), skip_nans=True)  #  This should generate a point cloud 
    rospy.sleep(1)

    print(type(generate))

    print("<<< NEW CLOUD START >>>")
    i = 0
    for point in generate:
        print("x: {x}, y: {y}, z: {z}".format(x=point[0], y=point[1], z=point[2]))
        i+=1
    print("<<< CLOUD END: {} POINTS >>>".format(i))


def main():
    rospy.init_node('kinect_point_subscriber_node', anonymous=True)
    subscriber = rospy.Subscriber('/camera/depth/points', PointCloud2, plc_callback) 

    rospy.spin()


if __name__ == "__main__":
    main()