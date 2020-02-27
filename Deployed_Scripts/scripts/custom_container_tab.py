#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

import os
from sortable_object_class import SortableObject
from toggle_frame_class import ToggledFrame

import xml.etree.ElementTree as ET
import json

import Tkinter as tk
import ttk
import tkMessageBox
import tkSimpleDialog


from Tkinter import *
from ttk import *



##  This class will be used to display the list of objects
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

        #  All this is for the toggle states
        self.m_object_checkbox_state_list = []  # List of checkbox states for individual objects
        self.m_object_checkbox_name_state_dict = dict()  # Dictionary of all object checkboxes 
        self.m_object_batch_relation_dict = dict()  #  This will help us identify the container that an object belongs to when it gets selected

        self.setup_widgets()  # Set up all graphical elements




    ##  This method will call all other methods which are responsible for setting up the tabs gui
    def setup_widgets(self):
        # self.setup_scrollable_frame()
        self.setup_scrollable_toggle()
        self.setup_container_info()
        


    ##  This method will display some information about the container
    def setup_container_info(self):
        self.selection_label_var = StringVar()  # Use this so it can be modified later
        self.selection_label_var.set(self.get_selected_object_str())  # Set to all selected items

        title_text = "Objects Selected for sorting:"
        self.selected_objects_title_label = tk.Label(self, text=title_text, font=("Helvetica", 12)).grid(row=1, column=3)
        self.selected_objects_label = tk.Label(self, textvariable=self.selection_label_var).grid(row=2, column=2, rowspan=3, columnspan=2, sticky="w")



    ##  TODO: Refactor this, pretty compact for a function atm
    ##  This method will update the labels showing the selected objects
    def update_selected_objects_label(self):
        self.selection_label_var.set(self.get_selected_object_str())


    ##  Return string of all selected objects
    def get_selected_object_str(self):
        final_str = ""

        if len(self.m_selected_objects_dict) > 0:
            for key, _ in self.m_selected_objects_dict.iteritems():
                final_str += str(key) + "\n\n"
        
        return final_str



    ##  This method will create all checkboxes within the current container tab
    def create_selection(self): 
        if RosContainerTab.__container_batches == None:
            error_name = "No Batches Found"
            error_msg = "No Batches exist. Unable to sort non-existant object batches."
            self.error_popup_msg(error_name, error_msg)
            return

        x = 0
        for _, name in enumerate(RosContainerTab.__container_batches.keys(), 1):
            batch_val = tk.IntVar()
            batch_name = str(name)
            
            self.selections[x] = tk.Checkbutton(self.frame_selection, text=str(name), variable=batch_val, onvalue=1, offvalue=0)
            self.selections[x].bind("<Button-1>", self.update_selected_objects)  # This waits for the item to be clicked, then it invokes update_selected_objects 
            self.selections[x].grid(row=x, column=0, sticky="w")  # Align the button on the left 
        
            self.m_checkbox_state_list.append(batch_val)  # Save reference of obj to look up state
            self.m_checkbox_name_state_dict.update({batch_name : batch_val})  # Add the object name : value pair into the dictionary

            x += 1



    ##  This method will setup the scrollview for the sortable objects list
    def setup_scrollable_frame(self):
        if type(self.m_all_objects) == NoneType:  #  TODO: Update this to use all batch objects
                return None

        #  Create a subframe where we can create a new canvas
        label = Label(self, text="Sortable Object Batches:", font=("Helvetica", 12)).grid(row=1, column=8)
        description = Label(self, text="A batch is a collection of objects.\nSelect a batch to add objects to the current container").grid(row=8, column=7)

        self.frame_canvas = tk.Frame(self)
        self.frame_canvas.grid(row=3, column=8, pady=(3,0), sticky='ne')
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



    ##  This method sets up a scrollable frame for the batch items
    def setup_scrollable_toggle(self):
        label = Label(self, text="Sortable Object Batches:", font=("Helvetica", 12)).grid(row=1, column=8)
        description = Label(self, text="A batch is a collection of objects.\nSelect a batch to add objects to the current container").grid(row=8, column=7)

        #  Create Frame where all subitems will be contained
        self.top_toggle_frame = tk.Frame(self)
        # self.top_toggle_frame.grid(row=3, column=8, rowspan=2, sticky='ne')
        self.top_toggle_frame.grid(row=3, column=8, pady=(2,0), rowspan=2, sticky='ne')
        self.top_toggle_frame.grid_rowconfigure(0, weight=1)
        self.top_toggle_frame.grid_columnconfigure(0, weight=1)
        self.top_toggle_frame.propagate(False)
    

        #  Create Canvas for Scrollbar and frame housing all items
        self.toggle_canvas = tk.Canvas(self.top_toggle_frame)
        self.toggle_canvas.grid(row=0, column=0, sticky='news')
        

        #  Create Vertical scollbar
        self.vscrollbar = tk.Scrollbar(self.top_toggle_frame, orient='vertical', command=self.toggle_canvas.yview)
        self.vscrollbar.grid(row=0, column=4, sticky="nsw")
        self.toggle_canvas.configure(yscrollcommand=self.vscrollbar.set)

        #  Create Frame for ToggledFrame objects
        self.toggled_frame = Frame(self.toggle_canvas)
        self.toggle_canvas.create_window((0,0), window=self.toggled_frame, anchor='ne')
        self.toggled_frame.grid(row=0, column=2)
        

         #  Add 9-by-1 CheckButtons to the frame
        rows = len(RosContainerTab.__container_batches)  # Get all batches that were parsed out in the xml
        rows_to_show = 5
        
        #  If there are less than 5 items, display all of them at once
        if rows < 5:
            rows_to_show = rows

        self.toggle_widgets = [ToggledFrame(None) for x in xrange(rows)]
        

        #  Create list so that we can scale the scroll view to the appropriate size of the buttons
        self.create_batch_toggle()  #  Create all the Toggles

        self.toggled_frame.update_idletasks()

        #  Resize the canvas frame to show exactly 1-by-5 Toggles and the scrollbar
        first5columns_width = max([self.toggle_widgets[j].winfo_width() for j in range(0, rows_to_show)])  # Find the widest element and set the minimum size to that
        first5rows_height = sum([self.toggle_widgets[i].winfo_height() for i in range(0, rows_to_show)])  #  Sum the height of all elements and set the size to that + the scrollbar
        
        self.top_toggle_frame.config(width=first5columns_width + self.vscrollbar.winfo_width(), height=first5rows_height)
        self.toggle_canvas.config(scrollregion=self.toggle_canvas.bbox("all"))  # Set the scroll region
        


    ##  This method will create the batch frames 
    def create_batch_toggle(self):
        if RosContainerTab.__container_batches == None:
            error_name = "No Batches Found"
            error_msg = "No Batches exist. Unable to sort non-existant object batches."
            self.error_popup_msg(error_name, error_msg)
            return
        
        x = 0
        for _, name in enumerate(RosContainerTab.__container_batches.keys(), 1):
            batch_name = str(name)

            self.toggle_widgets[x] = ToggledFrame(self.toggled_frame, collection_name=batch_name)
            self.toggle_widgets[x].grid(row=x, column=0, sticky="w")
            # self.toggle_widgets[x].pack(side="top", fill="x", expand=1, pady=2, padx=2, anchor="nw")
            self.create_toggle_subelements(batch_name, self.toggle_widgets[x])  # Create subelements

            x+=1
        


    ##  Create all subelemetns for a batch
    def create_toggle_subelements(self, batch_key, batch_widget):
        
        batch = RosContainerTab.__container_batches.get(batch_key)
        batch_objects = batch.get_all_object_names()

        #  If there are no objects in the batch (This should never be the case)
        if batch_objects == None:
            return

        #  Create checkboxes for all objects and add them to the subwidget 
        for obj_name in batch_objects:
            obj_val = tk.IntVar()

            selection = tk.Checkbutton(batch_widget.sub_frame, text=obj_name, variable=obj_val, onvalue=1, offvalue=0)
            selection.bind("<Button-1>", self.update_selected_toggled_objects)
            selection.pack()

            self.m_object_checkbox_state_list.append(obj_val)
            self.m_object_checkbox_name_state_dict.update({obj_name : obj_val})
            self.m_object_batch_relation_dict.update({obj_name : batch_key})

        



    ##  This method will handle the event when an object is selected, it will handle all adding and removing from dictionaries
    def update_selected_toggled_objects(self, event):
        obj_name = str(event.widget.cget("text"))
        state = self.m_object_checkbox_name_state_dict[obj_name].get()

        batch_name = self.m_object_batch_relation_dict.get(obj_name)
        batch = RosContainerTab.__container_batches.get(batch_name)

        check_widget = event.widget
        var_name = str(check_widget.cget("variable"))  # Get the state variable

        if state == 0:
            allocated_obj = batch.allocate_sortable_objects(1, self.m_container_pose, self.m_container_name)  # Get dictionary with object we can add

            if allocated_obj == None:
                error_name = "Error while allocating object"
                error_msg = "Object '%s' Already belongs to another container." % (obj_name)
                self.error_popup_msg(error_name, error_msg)
                self.m_object_checkbox_name_state_dict[obj_name].set(1)
                return
            
            self.add_batch_objects(allocated_obj)  # Use this method since it can handle any number of objects (0...n)

        else:
            self.remove_single_object_from_batch(obj_name, batch)

        self.update_selected_objects_label()


    ##  This method will take care of what objects are added and removed from the selected_objects dictionary
    def update_selected_objects(self, event):
        batch_name = str(event.widget.cget("text"))  # Get text field from widget
        state = self.m_checkbox_name_state_dict[batch_name].get()
        
        batch = RosContainerTab.__container_batches.get(batch_name)
        
        widget = event.widget
        var_name = str(widget.cget("variable"))

        if state == 0:  # if the check box wasn't selected before (implying that it is now)
            num_items = self.prompt_batch_size()
            
            if num_items == None:
                self.m_checkbox_name_state_dict[batch_name].set(1)  # Unselect the box
                return

            objs = batch.allocate_sortable_objects(num_items, self.m_container_pose, self.m_container_name)  # Get dictionary of objects we want to add
            
            if objs == None:
                error_name = "Exceeded Max object count!"
                error_msg = "Unable to select %s objects, only %s object(s) available!" % (num_items, batch.get_available_slots())
                self.error_popup_msg(error_name, error_msg)
                self.m_checkbox_name_state_dict[batch_name].set(1)
                return
            
            self.add_batch_objects(objs)

        else:  # Remove objects from batches allocation dict
            self.remove_batch_objects(batch)
        
        self.update_selected_objects_label()





    ##  This method adds multiple objects to the current container
    def add_batch_objects(self, obj_dict):
        for key, val in obj_dict.iteritems():
            self.add_object(key, val)


    ##  This method can be used to add an object to the m_selected_objects_dict dictionary
    def add_object(self, name, value):
        self.m_selected_objects_dict[str(name)] = value


    ##  This method allows us to remove a single item from a batch 
    def remove_single_object_from_batch(self, obj_key, batch):
        can_delete = False

        #  Check that container has access to item & that its part of the batch we're removing from
        for key, val in self.m_selected_objects_dict.iteritems():
            if val.m_assigned_container == self.m_container_name and val.m_batch_type == batch.m_type:
                can_delete = True

        if can_delete:
            self.remove_object(obj_key)
            batch.release_sortable_object(obj_key)


    ##  Remove objects from batch, only if they belong to this container 
    def remove_batch_objects(self, batch):
        rm_key_list = []

        for key, val in self.m_selected_objects_dict.iteritems():
            
            #  If container has access to the item & is part of the batch we're removing
            if val.m_assigned_container == self.m_container_name and val.m_batch_type == batch.m_type:
                rm_key_list.append(key)
        
        #  Need to remove items from tab dict (cant do it above since dict changes size)
        for keyi in rm_key_list:
            self.remove_object(keyi)
            batch.release_sortable_object(keyi)


    ##  This method will remove an object with object name 'key' from the dictionary
    def remove_object(self, key):
        if self.m_selected_objects_dict.has_key(key):
            del self.m_selected_objects_dict[key]



    ##  This method will create a popup if some error occures
    def error_popup_msg(self, error_name, error_msg):
        title = "Error: " + str(error_name)
        tkMessageBox.showerror(title, error_msg)        


    ##  Prompt the user to specify how many objects should be selected
    def prompt_batch_size(self):
        items = tkSimpleDialog.askinteger("Number of Items", "How many items should be selected?", parent = self)

        if items < 0:
            return None
        return items



    ##  This method will remove everything from the tab regarding sortable objects, called by notebook
    def clean_destroy(self):
        rm_key_list = []

        #  Move through all batches
        for _, batch in RosContainerTab.__container_batches.iteritems():
            
            #  Move through all selected items in this container
            for key, val in self.m_selected_objects_dict.iteritems():
                if val.m_assigned_container == self.m_container_name:
                    rm_key_list.append(key)
            for rkey in rm_key_list:
                self.remove_object(rkey)
                batch.release_sortable_object(rkey)





    

