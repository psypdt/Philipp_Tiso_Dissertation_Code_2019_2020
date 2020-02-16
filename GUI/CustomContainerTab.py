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
    
    ##  TODO: Implement default behaviour when a containers location is None, where should the arm go?
    def __init__(self, parent=None, i_container_name=None, i_container_position=None):
        ttk.Frame.__init__(self, parent)

        self.m_container_name = i_container_name
        self.m_container_pose = i_container_position  # The position where the container is located, None by default
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

        try:
            with open(full_path, 'rb') as xml_file:
                tree = ET.parse(xml_file)
        except Exception as e:
            print(e)
            print("Unable to find %s" % full_path)
            return None

        root = tree.getroot()

        items = root.getchildren()

        final_dict = dict()

        for item in items:
            name = item.attrib  # This is a dictionary of {key: obj_name}

            # print(name['name'])

            x = item.find('./position/x_pos').text
            y = item.find('./position/y_pos').text
            z = item.find('./position/z_pos').text

            obj_position = Pose()
            obj_position.position.x = float(x)
            obj_position.position.y = float(y)
            obj_position.position.z = float(z)

            if self.m_container_pose == None:
                print("Container has None pose")

            obj = SortableObject(obj_name=str(name['name']), obj_start=obj_position, obj_end=self.m_container_pose)
            final_dict[str(name['name'])] = obj

        return final_dict




    #  This method will call all other methods which are responsible for setting up the tabs gui
    def setup_widgets(self):
        self.setup_scrollable_frame()
        self.create_selection()
        self.setup_container_info()



    #  This method will display some information about the container
    def setup_container_info(self):
        text = "Container Position: \n" + str(self.m_container_pose.position)
        self.pos_label = ttk.Label(self, text=text).grid(row=1, column=1)



    #  This method will setup the scrollview for the sortable objects list
    def setup_scrollable_frame(self):
        #  Create a subframe where we can create a new canvas
        self.frame_canvas = tk.Frame(self)
        self.frame_canvas.grid(row=2, column=8, pady=(6,0), sticky='ne')
        self.frame_canvas.grid_rowconfigure(0, weight=1)
        self.frame_canvas.grid_columnconfigure(0, weight=1)
        self.frame_canvas.grid_propagate(False)

        #  Create canvas in subframe, where we will place the scroll bar and the item canvas
        self.canvas = tk.Canvas(self.frame_canvas)
        self.canvas.grid(row=0, column=0, sticky='news')

        #  Create Vertical scollbar
        self.vscrollbar = tk.Scrollbar(self.frame_canvas, orient='vertical', command=self.canvas.yview)
        self.vscrollbar.grid(row=0, column=2, sticky="ns")
        self.canvas.configure(yscrollcommand=self.vscrollbar.set)

        #  Create Frame that contains the boxes
        self.frame_selection = tk.Frame(self.canvas)
        self.canvas.create_window((0,0), window=self.frame_selection, anchor='ne')

        # Add 9-by-1 CheckButtons to the frame
        rows = len(self.m_all_objects)  # Get all objects that were contained in the xml
        rows_to_show = 9
        
        #  If there are less than 9 items, display all of them at once
        if rows < 9:
            rows_to_show = rows

        #  Create list so that we can scale the scroll view to the appropriate size of the buttons
        self.selections = [tk.Checkbutton() for x in xrange(rows)]  # Store all Checkbuttons
        
        self.create_selection()  #  Create all the CheckButtons
        self.frame_selection.update_idletasks()



        # Resize the canvas frame to show exactly 1-by-5 CheckButtons and the scrollbar
        first5columns_width = sum([self.selections[j].winfo_width() for j in range(0, 1)])
        first5rows_height = sum([self.selections[i].winfo_height() for i in range(0, rows_to_show)])
        
        self.frame_canvas.config(width=first5columns_width + self.vscrollbar.winfo_width(), height=first5rows_height)
        self.canvas.config(scrollregion=self.canvas.bbox("all"))






    #  This method will create all checkboxes within the current container tab
    def create_selection(self): 
        x = 0
        for _, name in enumerate(self.m_all_objects.keys(), 1):
            obj_val = tk.IntVar()
            obj_name = str(name)
            
            self.selections[x] = tk.Checkbutton(self.frame_selection, text=str(name), variable=obj_val, onvalue=1, offvalue=0)
            self.selections[x].bind("<Button-1>", self.update_selected_objects)  # This waits for the item to be clicked, then it invokes update_selected_objects 
            self.selections[x].grid(row=x, column=0)
        
            self.m_checkbox_state_list.append(obj_val)  # Save reference of obj to look up state
            self.m_checkbox_name_state_dict.update({obj_name : obj_val})  # Add the object name : value pair into the dictionary

            x += 1
        


    #  This method will take care of what objects are added and removed from the selected_objects dictionary
    def update_selected_objects(self, event):
        name = str(event.widget.cget("text"))  # Get text field from widget
        state = self.m_checkbox_name_state_dict[name].get()

        if state == 0:  # if the check box wasn't selected before (implying that it is now)
            obj = self.m_all_objects.get(name)
            self.add_object(name, obj)
        else:
            self.remove_object(name)



    #  This method can be used to add an object to the m_selected_objects_dict dictionary
    def add_object(self, name, value):
        self.m_selected_objects_dict[str(name)] = value


    #  This method will remove an object with object name 'key' from the dictionary
    def remove_object(self, key):
        if self.m_selected_objects_dict.has_key(key):
            del self.m_selected_objects_dict[key]


