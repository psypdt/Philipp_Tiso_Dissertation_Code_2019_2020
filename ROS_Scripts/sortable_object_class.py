#! /usr/bin/env python

from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose
from intera_examples.msg import SortableObjectMessage
import StringIO


class SortableObject(object):
    static_object_counter = 1  # Static counter to track number of objects

    
    def __init__(self, obj_name=None, obj_position=None, container_position=None, container_name=None, batch_type=None, gripper_distance=0.0065):
        #  If no name was given, create one to make the object unique
        if obj_name == None:
            obj_name = "sortable_object" + str(SortableObject.static_object_counter)
            SortableObject.static_object_counter += 1  # Increment number of objects if no name was given

        self.m_name = obj_name
        self.m_batch_type = batch_type
        self.m_assigned_container = container_name
        self.gripper_distance = gripper_distance

        self.m_start_pose = obj_position  # Where object is initially located in the work space
        self.m_container_pose = container_position  # The location of the container where the object should be placed


        if self.m_start_pose == None:
            self.m_start_pose = Pose()
        
        if self.m_container_pose == None:
            self.m_container_pose = Pose()


    
    #  This method can be invoked to get a SortableObjectMessage object from the current object instance
    def to_sortableObjectMessage(self):
        return SortableObjectMessage(self.m_name, self.m_assigned_container, self.m_start_pose, self.m_container_pose, True, self.gripper_distance)