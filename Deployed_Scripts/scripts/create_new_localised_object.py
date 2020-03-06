#! /usr/bin/env python


import Tkinter as tk
import ttk

import ros_xml_manipulation as RXM
import xml.etree.ElementTree as ET

import tkMessageBox

from Tkinter import *
from ttk import *


##  This class displays the instructions to the user when a new object is to be added & localised
##  This class handles all the xml writing calls
class ObjectLocationInputBox(ttk.Frame):

    ##  Need the Notebook to call the update method for the batch objects
    def __init__(self, parent=None, obj_pose=None, notebook=None, is_object=True):
        ttk.Frame.__init__(self, parent)
        
        self.new_pose = obj_pose.pose
        self.p_notebook = notebook  # The partents notebook which will update the gui
        self.gripper_dist = obj_pose.gripper_position

        #  Setup relevant widgets when adding a new sortable object
        if is_object:
            self.setup_object_widgets()
        else:
            self.setup_container_widgets()

        self.pack()


    def setup_object_widgets(self):
        #  Create label and entry field for object name
        self.name_label = ttk.Label(self, text="Object Name (Hammer, Apple, etc.)")
        self.name_entry = Entry(self)

        #  Create label and entry field for object type
        self.type_label = ttk.Label(self, text="Object Type (Tool, Fruit, etc.)")
        self.type_entry = Entry(self)        

        self.name_label.grid(row=0, column=0, sticky="w", pady=2)
        self.type_label.grid(row=1, column=0, sticky="w", pady=2)

        self.name_entry.grid(row=0, column=1, pady=2)
        self.type_entry.grid(row=1, column=1, pady=2)

        #  Define behaviour for buttons (Done, Cancel)
        self.done_button = Button(self, text="Done", command=self.write_new_object_xml)
        self.cancel_button = Button(self, text="Cancel", command=self.master.destroy)

        self.done_button.grid(row=3, column=0)
        self.cancel_button.grid(row=3, column=1)



    ##  Setup widgets if the user is adding a container
    def setup_container_widgets(self):
        self.name_label = ttk.Label(self, text="Name of the new container")
        self.name_entry = Entry(self)

        self.name_label.grid(row=0, column=0, sticky="w", pady=2)
        self.name_entry.grid(row=1, column=0, sticky="w", pady=2)

        self.done_button = Button(self, text="Done", command=self.write_new_container_xml)
        self.cancel_button = Button(self, text="Cancel", command=self.master.destroy)

        self.done_button.grid(row=2, column=0)
        self.cancel_button.grid(row=2, column=1)




    ##  This method will write a new object into an xml file that stores all existing objects
    def write_new_object_xml(self):
        new_name = str(self.name_entry.get())
        new_type = str(self.type_entry.get())

        if not new_name or not new_type or new_name == None or new_type == None:
            return

        #  Check if object already exists
        existing_items = self.p_notebook.get_all_sortable_object_names()
        
        if new_name.lower() in existing_items:
            tkMessageBox.showwarning("Object overwrite!", "Warning: the object %s exists already!" % new_name)
            return

        
        RXM.append_to_xml_file(filename="object_positions", name=new_name, obj_type=new_type, pose=self.new_pose, gripper_dist=self.gripper_dist)

        if self.p_notebook != None:
            self.p_notebook.update_batch_contents()

        self.master.destroy()



    ##  This method will write the newly added container into an xml and make the notebook update itself to give us access to the new container
    def write_new_container_xml(self):
        new_name = str(self.name_entry.get())

        if not new_name or new_name == None:
            return

        RXM.append_to_xml_file(filename="container_positions", name=new_name, obj_type=None, pose=self.new_pose)
        
        if self.p_notebook != None:
            self.p_notebook.read_new_container()  # Read the newly added container and make it available

        self.master.destroy()