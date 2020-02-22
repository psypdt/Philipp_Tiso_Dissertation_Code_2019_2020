#! /usr/bin/env python

from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose
from intera_examples.msg import SortableObjectMessage
import json
import simplejson
from json import JSONEncoder
import StringIO


class SortableObject(object):
    static_object_counter = 1  # Static counter to track number of objects


    #  This method will generate JSON when the object needs to be serialised
    def __json__(self):
        return {
            'm_name': self.m_name,
            'm_start_pose': self.m_start_pose,
            'm_container_pose': self.m_container_pose,
        }

    for_json = __json__  # To support simlejson

    #  cls is the class which is currently being used (don't need to pass it as arg when using this method)
    #  This method will convert from json back to an object
    @classmethod
    def from_json(cls, json):
        obj = cls()
        res_dict = eval(json)  # Create dictionary where values can be extracted from
        obj.m_name = res_dict["m_name"]
        obj.m_start_pose = res_dict['m_start_pose']
        obj.m_container_pose = res_dict['m_container_pose']
        
        return obj


    
    def __init__(self, obj_name="sortable", obj_position=None, container_position=None, container_name=None, batch_type=None):
        self.m_name = obj_name + str(SortableObject.static_object_counter)
        self.m_batch_type = batch_type
        self.m_assigned_container = container_name

        self.m_start_pose = obj_position  # Where object is initially located in the work space
        self.m_container_pose = container_position  # The location of the container where the object should be placed

        SortableObject.static_object_counter += 1  # Increment number of objects

        if self.m_start_pose == None:
            self.m_start_pose = Pose()
        
        if self.m_container_pose == None:
            self.m_container_pose = Pose()


    
    #  Iterator for object such that it can be used in for x in list
    def __iter__(self):
        return self


    
    #  This method can be invoked to get a SortableObjectMessage object from the current object instance
    def to_sortableObjectMessage(self):
        return SortableObjectMessage(self.m_name, self.m_start_pose, self.m_container_pose)