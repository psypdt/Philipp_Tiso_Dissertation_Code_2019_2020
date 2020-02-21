#! usr/bin/env python


import rospy
from psypdt_dissertation.msg import SortableObjectMessage

from sortable_object_class import SortableObject



#  This class is a collection of sortable objects all of which are the same type
class SortableBatch():
    
    def __init__(self, i_object_type):
        self.m_type = i_object_type  # The type of object (screw, hammer, bottle, etc.)
        self.m_available_objects_dict = dict()  # Dictionary of objects which can be distributed to containers
        self.m_allocated_objects_dict = dict()  # Dictionary of allocated objects



    def get_available_slots(self):
        return len(self.m_available_objects_dict)



    ##  This method allows us to allocate n SortableObjects to a specific container returned as a list
    def allocate_sortable_objects(self, n, container_pose, container_name):
        if n > len(self.m_available_objects_dict):
            return None

        res_dict = dict()

        for i in range(n):
            key, val = self.m_available_objects_dict.popitem()
            self.m_allocated_objects_dict.update({key : val})

            val.m_container_pose = container_pose
            val.m_assigned_container = container_name
            res_dict.update({key : val})

            print "Allocated %s to Container: %s" % (key, container_name)
            
        return res_dict


    ##  Remove single sortable object, place it back into available dict
    def release_sortable_object(self, key):
        if self.m_allocated_objects_dict.has_key(key):
            val = self.m_allocated_objects_dict.pop(key)
            
            val.m_assigned_container = None
            val.m_container_pose = None

            self.m_available_objects_dict.update({key : val})
