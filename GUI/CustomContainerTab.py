#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

import os
from sorting_object_class import SortableObject

import xml.etree.ElementTree as ET
import json

import Tkinter as tk
import ttk


from Tkinter import *
from ttk import *


#  This class will be used to display the list of objects
class RosContainerTab(ttk.Frame):
    
    def __init__(self, parent=None, i_container=None, i_location=None):
        ttk.Frame.__init__(self, parent)

        self.m_container_pose = i_location  # The position where the container is located
        self.m_container_name = i_container
        self.m_selected_objects_dict = {}  # Dictionary containing all selected objects
        self.m_all_objects = self.read_all_objects()  # Dictionary containing all objects from xml file (name : SortableObject)
        self.m_checkbox_state_list = []  # List containing all checkbox states
        self.m_checkbox_name_state_dict = dict()  # Dictionary for all checkboxes

        self.setup_widgets()




    #  This method will read all objects from an xml and will return a dictionary containing said objects (including name and postition)
    def read_all_objects(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        suffix = ".xml"
        filename = "object_positions"
        
        full_path = os.path.join(dir_path, filename + suffix)

        with open(full_path, 'rb') as xml_file:
            tree = ET.parse(xml_file)
        
        root = tree.getroot()

        items = root.getchildren()

        final_dict = dict()

        for item in items:
            name = item.attrib  # This is a dictionary of {key: obj_name}

            # print(name['name'])

            x = item.find('./position/x_pos').text
            y = item.find('./position/y_pos').text
            z = item.find('./position/z_pos').text

            start = Pose()
            start.position.x = float(x)
            start.position.y = float(y)
            start.position.z = float(z)

            obj = SortableObject(obj_name=str(name['name']), obj_start=start, obj_end=self.m_container_pose)
            final_dict[str(name['name'])] = obj

        return final_dict




    #  This method will call all other methods which are responsible for setting up the tabs gui
    def setup_widgets(self):
        self.create_selection()




    #  This method will create all checkboxes within the current container tab
    def create_selection(self):
        x = 1
        for _, name in enumerate(self.m_all_objects.keys(), 1):
            obj_val = tk.IntVar()
            obj_name = str(name)
            
            checked = tk.Checkbutton(self, text=str(name), variable=obj_val, onvalue=1, offvalue=0)
            checked.bind("<Button-1>", self.update_selected_objects)  # This waits for the item to be clicked, then it invokes update_selected_objects 
            checked.grid(row=x, column=9)
        
            self.m_checkbox_state_list.append(obj_val)  # Save reference of obj to look up state
            self.m_checkbox_name_state_dict.update({obj_name : obj_val})  # Add the object name : value pair into the dictionary
            
            x += 1
        


    #  This method will take care of what objects are added and removed from the selected_objects dictionary
    def update_selected_objects(self, event):
        name = str(event.widget.cget("text"))  # Get text field from widget
        state = self.m_checkbox_name_state_dict[name].get()
        
        # print("Tab is %s" % self.m_container_name)

        if state == 0:  # if the check box wasn't selected before (implying that it is now)
            obj = self.m_all_objects.get(name)
            self.add_object(name, obj)
        else:
            self.remove_object(name)

        for e in self.m_selected_objects_dict.values():
            print(type(e))



    #  This method can be used to add an object to the m_selected_objects_dict dictionary
    def add_object(self, name, value):
        self.m_selected_objects_dict[str(name)] = value


    #  This method will remove an object with object name 'key' from the dictionary
    def remove_object(self, key):
        if self.m_selected_objects_dict.has_key(key):
            del self.m_selected_objects_dict[key]
