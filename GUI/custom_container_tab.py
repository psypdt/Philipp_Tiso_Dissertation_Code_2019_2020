#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

import os
from sortable_object_class import SortableObject

import xml.etree.ElementTree as ET
import json

import Tkinter as tk
import ttk
import tkMessageBox
import tkSimpleDialog


from Tkinter import *
from ttk import *



#  This class will be used to display the list of objects
class RosContainerTab(ttk.Frame):
    
    __container_batches = dict()  # Batches should be shared across all tab instances


    ##  TODO: Implement default behaviour when a containers location is None, where should the arm go?
    def __init__(self, parent=None, i_container_name=None, i_container_position=None, i_batches=None):
        ttk.Frame.__init__(self, parent)

        self.m_container_name = i_container_name
        self.m_container_pose = i_container_position  # The position where the container is located, None by default
        RosContainerTab.__container_batches = i_batches

        self.m_selected_objects_dict = {}  # Dictionary containing all selected objects
        self.m_all_objects = dict()  # Dictionary containing all objects from xml file (name : SortableObject)
        self.m_checkbox_state_list = []  # List containing all checkbox states
        self.m_checkbox_name_state_dict = dict()  # Dictionary for all checkboxes

        self.setup_widgets()  # Set up all graphical elements




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
            error = "Unable to find objects in file: " + str(filename + str(suffix)) + "\n\nAt location: " + str(full_path)
            self.error_popup_msg("Can't find object file", error)
            return None

        root = tree.getroot()
        items = root.getchildren()

        final_dict = dict()

        for item in items:
            name = str(item.attrib['name'])  # This is a dictionary of {key: obj_name}

            x = item.find('./position/x_pos').text
            y = item.find('./position/y_pos').text
            z = item.find('./position/z_pos').text

            obj_position = Pose()
            obj_position.position.x = float(x)
            obj_position.position.y = float(y)
            obj_position.position.z = float(z)

            if self.m_container_pose == None:
                error = "The container: " + str(self.m_container_name) + " doesn't have a physical position! Check the container xml file."
                self.error_popup_msg("Container doesn't exist", error)
                print("Container has None pose")

            obj = SortableObject(obj_name=str(name), obj_position=obj_position, container_position=self.m_container_pose)
            final_dict[str(name)] = obj

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
        if type(self.m_all_objects) == NoneType:  # TODO: Update this to use all batch objects
                return None

        #  Create a subframe where we can create a new canvas
        label = Label(self, text="Sortable Object Batches:", font=("Helvetica", 12)).grid(row=1, column=8)
        self.frame_canvas = tk.Frame(self)
        self.frame_canvas.grid(row=2, column=8, pady=(5,0), sticky='ne')
        self.frame_canvas.grid_rowconfigure(0, weight=1)
        self.frame_canvas.grid_columnconfigure(0, weight=1)
        self.frame_canvas.grid_propagate(False)

        #  Create canvas in subframe, where we will place the scroll bar and the item canvas
        self.canvas = tk.Canvas(self.frame_canvas)
        self.canvas.grid(row=0, column=0, sticky='news')

        #  Create Vertical scollbar
        self.vscrollbar = tk.Scrollbar(self.frame_canvas, orient='vertical', command=self.canvas.yview)
        self.vscrollbar.grid(row=0, column=3, sticky="nsw")
        self.canvas.configure(yscrollcommand=self.vscrollbar.set)

        #  Create Frame that contains the boxes
        self.frame_selection = tk.Frame(self.canvas)
        self.canvas.create_window((0,0), window=self.frame_selection, anchor='ne')

        #  Add 9-by-1 CheckButtons to the frame
        rows = len(RosContainerTab.__container_batches)  # Get all batches that were parsed out in the xml
        rows_to_show = 9
        
        #  If there are less than 9 items, display all of them at once
        if rows < 9:
            rows_to_show = rows

        #  Create list so that we can scale the scroll view to the appropriate size of the buttons
        self.selections = [tk.Checkbutton() for x in xrange(rows)]  # Store all Checkbuttons
        
        self.create_selection()  #  Create all the CheckButtons
        self.frame_selection.update_idletasks()


        #  Resize the canvas frame to show exactly 1-by-5 CheckButtons and the scrollbar
        first5columns_width = max([self.selections[j].winfo_width() for j in range(0, rows_to_show)])  # Find the widest element and set the minimum size to that
        first5rows_height = sum([self.selections[i].winfo_height() for i in range(0, rows_to_show)])  #  Sum the height of all elements and set the size to that + the scrollbar
        
        self.frame_canvas.config(width=first5columns_width + self.vscrollbar.winfo_width(), height=first5rows_height)
        self.canvas.config(scrollregion=self.canvas.bbox("all"))  # Set the scroll region





    #  This method will create all checkboxes within the current container tab
    def create_selection(self): 
        if RosContainerTab.__container_batches == None:
            return

        x = 0
        # for _, name in enumerate(self.m_all_objects.keys(), 1):
        for _, name in enumerate(RosContainerTab.__container_batches.keys(), 1):
            batch_val = tk.IntVar()
            batch_name = str(name)
            
            self.selections[x] = tk.Checkbutton(self.frame_selection, text=str(name), variable=batch_val, onvalue=1, offvalue=0)
            self.selections[x].bind("<Button-1>", self.update_selected_objects)  # This waits for the item to be clicked, then it invokes update_selected_objects 
            self.selections[x].grid(row=x, column=0, sticky="w")  # Align the button on the left 
        
            self.m_checkbox_state_list.append(batch_val)  # Save reference of obj to look up state
            self.m_checkbox_name_state_dict.update({batch_name : batch_val})  # Add the object name : value pair into the dictionary

            x += 1
        


    #  This method will take care of what objects are added and removed from the selected_objects dictionary
    def update_selected_objects(self, event):
        batch_name = str(event.widget.cget("text"))  # Get text field from widget
        state = self.m_checkbox_name_state_dict[batch_name].get()
        
        batch = RosContainerTab.__container_batches.get(batch_name)
        
        widget = event.widget
        var_name = str(widget.cget("variable"))

        if state == 0:  # if the check box wasn't selected before (implying that it is now)
            num_items = self.prompt_batch_size()
            
            if num_items == None:
                rev_val = self.m_checkbox_name_state_dict[batch_name].set(1)  # Unselect the box
                return

            objs = batch.allocate_sortable_objects(num_items, self.m_container_pose, self.m_container_name)  # Get dictionary of objects we want to add
            print objs
            if objs == None:
                error_name = "Exceeded Max object count!"
                error_msg = "Unable to select %s objects when only %s are available!" % (num_items, batch.get_available_slots())
                self.error_popup_msg(error_name, error_msg)
                return
            
            self.add_batch_objects(objs)

        else:  # Remove objects from batches allocation dict
            self.remove_batch_objects(batch)




    ##  This method adds multiple objects to the current container
    def add_batch_objects(self, obj_dict):
        for key, val in obj_dict.iteritems():
            self.add_object(key, val)


    ## This method can be used to add an object to the m_selected_objects_dict dictionary
    def add_object(self, name, value):
        self.m_selected_objects_dict[str(name)] = value



    ##  Remove objects from batch, only if they belong to this container 
    def remove_batch_objects(self, batch):
        rm_key_list = []
        for key, val in self.m_selected_objects_dict.iteritems():
            if val.m_assigned_container == self.m_container_name:  # TODO: Awful way of doing this, need to find a better way to check if obj belongs to tab
                batch.release_sortable_object(key)
                rm_key_list.append(key)
        
        # Need to remove items from tab dict (cant do it above since dict changes size)
        for key in rm_key_list:
            self.remove_object(key)


    #  This method will remove an object with object name 'key' from the dictionary
    def remove_object(self, key):
        if self.m_selected_objects_dict.has_key(key):
            del self.m_selected_objects_dict[key]



    #  This method will create a popup if some error occures
    def error_popup_msg(self, error_name, error_msg):
        title = "Error: " + str(error_name)
        tkMessageBox.showerror(title, error_msg)        


    ##  Prompt the user to specify how many objects should be selected
    def prompt_batch_size(self):
        items = tkSimpleDialog.askinteger("Number of Items", "How many items should be selected?", parent = self)

        if items < 0:
            return None

        return items



    

