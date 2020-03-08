#! usr/bin/env python


import rospy
from intera_examples.msg import SortableObjectMessage
from sortable_object_class import SortableObject



#  This class is a collection of sortable objects all of which are the same type
class SortableBatch():
    
    def __init__(self, i_object_type):
        self.m_type = i_object_type  # The type of object (screw, hammer, bottle, etc.)
        self.m_available_objects_dict = dict()  # Dictionary of objects which can be distributed to containers
        self.m_allocated_objects_dict = dict()  # Dictionary of allocated objects



    def get_available_slots(self):
        return len(self.m_available_objects_dict)


    def get_available_object_names(self):
        return self.m_available_objects_dict.keys()

    
    ##  This method returns every object that exists within the batch
    def get_all_object_names(self):
        allocated_names = self.m_allocated_objects_dict.keys()
        available_names = self.m_available_objects_dict.keys()

        final_list = None

        if len(allocated_names) > 0 and len(available_names) > 0:
            final_list = allocated_names + available_names
        elif len(allocated_names) <= 0:
            final_list = available_names
        elif len(available_names) <=0:
            final_list = allocated_names
        
        return final_list


    ##  This method allows us to allocate n SortableObjects to a specific container returned as a list
    def allocate_sortable_objects(self, object_name=None, container_pose=None, container_name=None):
        res_dict = dict()  # Dictionary of all available items

        #  Allocate a specific item
        if object_name != None and self.m_available_objects_dict.has_key(object_name):
            val = self.m_available_objects_dict.pop(object_name)
            self.m_allocated_objects_dict.update({object_name : val})

            val.m_container_pose = container_pose
            val.m_assigned_container = container_name
            res_dict.update({object_name : val})
            # print "Allocated %s to Container: %s" % (object_name, container_name)
            return res_dict
        
        return None



    ##  Remove single sortable object, place it back into available dict
    def release_sortable_object(self, key):
        if self.m_allocated_objects_dict.has_key(key):
            val = self.m_allocated_objects_dict.pop(key)
            
            val.m_assigned_container = None
            val.m_container_pose = None

            self.m_available_objects_dict.update({key : val})