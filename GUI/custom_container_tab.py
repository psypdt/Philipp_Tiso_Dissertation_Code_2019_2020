#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

import os
from sortable_object_class import SortableObject
from toggle_frame_class import ToggledBatchFrame

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

        #  All this is for the toggle states
        self.m_object_checkbox_state_list = []  # List of checkbox states for individual objects
        self.m_object_checkbox_name_state_dict = dict()  # Dictionary of all object checkboxes 
        self.m_object_batch_relation_dict = dict()  #  This will help us identify the container that an object belongs to when it gets selected
        self.m_batchKey_toggle_frame_dict = dict()  #  This is a way for us to get a toggled frame via its batch name

        self.setup_widgets()  # Set up all graphical elements




    ##  This method will call all other methods which are responsible for setting up the tabs gui
    def setup_widgets(self):
        self.setup_scrollable_toggle()
        self.setup_container_info()
        

    ##  This method will display some information about the container
    def setup_container_info(self):
        # self.selection_label_var = StringVar()  # Use this so it can be modified later
        # self.selection_label_var.set(self.get_selected_object_str())  # Set to all selected items

        title_text = "Objects Select for container %s:" % self.m_container_name
        self.selected_objects_title_label = tk.Label(self, text=title_text, font=("Helvetica", 12, 'bold')).grid(row=1, column=2)
        # self.selected_objects_label = tk.Label(self, textvariable=self.selection_label_var).grid(row=2, column=2, rowspan=3, columnspan=2, sticky="w")
        self.textbox_frame = tk.Frame(self)
        self.textbox_frame.grid(row=2, column=2, rowspan=3, columnspan=2, sticky="w")
        self.textbox_frame.rowconfigure(0, weight=1)
        self.textbox_frame.columnconfigure(0, weight=1)
        
        #  Create horizontal scroll
        self.horizontal_textbox_scroll = Scrollbar(self.textbox_frame, orient="horizontal")
        self.horizontal_textbox_scroll.pack(side="bottom", fill="x")

        #  Create vertical scroll
        self.vertical_textbox_scroll = Scrollbar(self.textbox_frame, orient="vertical")
        self.vertical_textbox_scroll.pack(side="right", fill="y")

        #  Create textbox for user to see selected objects
        self.selected_objects_textbox = Text(self.textbox_frame, wrap="none", width=25, height=20, font=("Helvetica", 8, 'bold'),
                                                yscrollcommand=self.vertical_textbox_scroll.set, xscrollcommand=self.horizontal_textbox_scroll.set)
        self.selected_objects_textbox.config(state="disabled")
        self.selected_objects_textbox.pack(side="left", fill="y")

        #  Configure scroll commands
        self.horizontal_textbox_scroll.config(command=self.selected_objects_textbox.xview)
        self.vertical_textbox_scroll.config(command=self.selected_objects_textbox.yview)
        
        



    ##  This method will update the text widget displaying all selected items
    def update_selected_objects_label(self):
        self.selected_objects_textbox.config(state="normal")  # Set state to normal so we can delete old data & insert new data
        
        new_text = self.get_selected_object_str()
        
        self.selected_objects_textbox.delete('1.0', END)  # Remove old content from start (1.0) to the end
        self.selected_objects_textbox.insert('1.0', new_text)
        self.selected_objects_textbox.config(state="disabled")  # Prevent user from editing text
        self.selected_objects_textbox.pack(side="left", fill="y")



    ##  Return string of all selected objects
    def get_selected_object_str(self):
        final_str = ""
        item_list = []

        if len(self.m_selected_objects_dict) > 0:
            #  Get all items
            for key, _ in self.m_selected_objects_dict.iteritems():
                item_list.append(str(key).capitalize())
            item_list.sort()

            for obj in item_list:
                final_str += obj + "\n\n"
        
        return final_str




    ##  This method sets up a scrollable frame for the batch items
    def setup_scrollable_toggle(self):
        label = Label(self, text="Sortable Object Batches:", font=("Helvetica", 12, 'bold')).grid(row=1, column=8)
        
        #  Create Canvas for Scrollbar and frame housing all items
        self.toggle_canvas = Canvas(self, width=20, height=25)
        self.toggle_canvas.grid(row=2, column=8, rowspan=4, sticky='news')
        

        #  Create Frame for ToggledBatchFrame objects
        self.toggled_frame = Frame(self.toggle_canvas)
        self.toggle_canvas.create_window((0,0), window=self.toggled_frame, anchor='nw')
        # self.toggled_frame.grid(row=0, column=2)  # Removing this fixes the frame size, but causes expaned selection to overflow onto the left side
        
        #  Create scroll bars
        self.x_scroll_toggled_canvas = Scrollbar(self.toggle_canvas, orient="horizontal")
        self.y_scroll_toggled_canvas = Scrollbar(self.toggle_canvas, orient="vertical")

        #  Set the position of the scrollbars
        self.x_scroll_toggled_canvas.pack(side="bottom", fill="x")
        self.y_scroll_toggled_canvas.pack(side="right", fill="y")

         #  Add 9-by-1 CheckButtons to the frame
        rows = len(RosContainerTab.__container_batches)  # Get all batches that were parsed out in the xml
        rows_to_show = 5
        
        #  If there are less than 5 items, display all of them at once
        if rows < 5:
            rows_to_show = rows

        self.toggle_widgets = [ToggledBatchFrame(None) for x in xrange(rows)]
        

        #  Create list so that we can scale the scroll view to the appropriate size of the buttons
        self.create_batch_toggle()  #  Create all the Toggles
        self.toggled_frame.update_idletasks()


        #  Configure scollbars and scroll regions, this must be done here since we need to know the max size of the elements we added
        self.toggle_canvas.config(scrollregion=self.toggle_canvas.bbox('all'))
        self.toggle_canvas.config(xscrollcommand=self.x_scroll_toggled_canvas.set, yscrollcommand=self.y_scroll_toggled_canvas.set)
        self.x_scroll_toggled_canvas.config(command=self.toggle_canvas.xview)
        self.y_scroll_toggled_canvas.config(command=self.toggle_canvas.yview)





    ##  Use to update the canvas scrollbars when a new item is added
    def update_canvas_scroll(self):
        self.toggled_frame.update_idletasks()  # Refresh all the items in the frame for toggled items
        self.toggle_canvas.config(scrollregion=self.toggle_canvas.bbox('all'))  # Reconfigure region
        self.toggle_canvas.yview('moveto', '1.0')  # Move scroll to top
        self.toggle_canvas.xview('moveto', '0.0')  # Move to left most side





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
            self.toggle_widgets[x] = ToggledBatchFrame(self.toggled_frame, collection_name=batch_name)
            self.toggle_widgets[x].grid(row=x, column=0, sticky="w")
            # self.toggle_widgets[x].pack(side="top", fill="x", expand=1, pady=2, padx=2, anchor="nw")
            self.create_toggle_subelements(batch_name, self.toggle_widgets[x])  # Create subelements
            self.m_batchKey_toggle_frame_dict.update({batch_name : self.toggle_widgets[x]})  # Save toggled element so we can add new objects to it later

            x+=1
        

    ##  Create all subelemetns for a batch, set is_existing_batch to True if the batch already exists
    def create_toggle_subelements(self, batch_key, batch_widget, is_existing_batch=False):
        batch = RosContainerTab.__container_batches.get(batch_key)
        batch_objects = batch.get_all_object_names()

        #  If there are no objects in the batch (This should never be the case)
        if batch_objects == None:
            return

        #  Add a newly located object into an existing batch, "located drill which belongs into Tools"
        if is_existing_batch:
            outdated_batch_objects = self.m_object_checkbox_name_state_dict.keys()  # Get the name of all items currently displayed
            new_obj_name_list = list(set(batch_objects) - set(outdated_batch_objects))
            
            #  TODO: Inform the user that the object exists already, maybe ask if they wish to overwrite its possition
            if len(new_obj_name_list) <= 0:  # The object already exists
                return
            
            new_obj_name = new_obj_name_list[0]
            new_obj_val = tk.IntVar()

            new_selection = tk.Checkbutton(batch_widget.sub_frame, text=new_obj_name, variable=new_obj_val, onvalue=1, offvalue=0)
            new_selection.bind("<Button-1>", self.update_selected_toggled_objects_state)
            # new_selection.pack(anchor="w")
            new_selection.pack()

            self.m_object_checkbox_state_list.append(new_obj_val)
            self.m_object_checkbox_name_state_dict.update({new_obj_name : new_obj_val})
            self.m_object_batch_relation_dict.update({new_obj_name : batch_key})
            return
        

        #  Create checkboxes for all objects and add them to the subwidget 
        for obj_name in batch_objects:
            obj_val = tk.IntVar()

            selection = tk.Checkbutton(batch_widget.sub_frame, text=obj_name, variable=obj_val, onvalue=1, offvalue=0)
            selection.bind("<Button-1>", self.update_selected_toggled_objects_state)
            selection.pack(anchor="w")

            self.m_object_checkbox_state_list.append(obj_val)
            self.m_object_checkbox_name_state_dict.update({obj_name : obj_val})
            self.m_object_batch_relation_dict.update({obj_name : batch_key})


        
    ##  This method will handle the event when an object is selected, it will handle all adding and removing from the selected_objects dictionarie for selected sortable_items
    def update_selected_toggled_objects_state(self, event):
        obj_name = str(event.widget.cget("text"))
        state = self.m_object_checkbox_name_state_dict[obj_name].get()

        batch_name = self.m_object_batch_relation_dict.get(obj_name)
        batch = RosContainerTab.__container_batches.get(batch_name)

        check_widget = event.widget
        var_name = str(check_widget.cget("variable"))  # Get the state variable

        if state == 0:
            allocated_obj = batch.allocate_sortable_objects(object_name=obj_name, container_pose=self.m_container_pose, container_name=self.m_container_name)  # Get dictionary with object we can add

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



    ##  This method will add any new objects and batches to the current frame
    def refresh_toggled_batches(self, modified_batch_name):
        existing_toggles = [name.batch_text.lower() for name in self.toggle_widgets]  # Need lower() since displayed text doesn't match the xml which is all lowercase

        #  Check if batch exists or if we need a new one
        if modified_batch_name in existing_toggles:
            toggled_widget = self.m_batchKey_toggle_frame_dict.get(modified_batch_name)
            
            #  Add to the existing batch
            self.create_toggle_subelements(modified_batch_name, toggled_widget, is_existing_batch=True)

        else:
            new_toggle = ToggledBatchFrame(self.toggled_frame, collection_name=modified_batch_name)
            x = len(self.toggle_widgets)  # Use this to find where the new toggle should be placed
    
            self.toggle_widgets.append(new_toggle)
            self.toggle_widgets[x].grid(row=x, column=0, sticky="w")
            
            self.create_toggle_subelements(modified_batch_name, self.toggle_widgets[x])
            self.m_batchKey_toggle_frame_dict.update({modified_batch_name : self.toggle_widgets[x]})

        self.update_canvas_scroll()  # Update scrollbars




    ##  This method adds multiple objects to the current container
    def add_batch_objects(self, obj_dict):
        for key, val in obj_dict.iteritems():
            self.add_object(key, val)


    ##  This method can be used to add an object to the m_selected_objects_dict dictionary
    def add_object(self, name, value):
        self.m_selected_objects_dict[str(name)] = value




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


    ##  This method allows us to remove a single item from a batch 
    def remove_single_object_from_batch(self, obj_key, batch):
        can_delete = False

        #  Check that container has access to item & that its part of the batch we're removing from
        for key, val in self.m_selected_objects_dict.iteritems():
            if val.m_assigned_container == self.m_container_name and val.m_batch_type == batch.m_type:
                can_delete = True
                break

        if can_delete:
            self.remove_object(obj_key)
            batch.release_sortable_object(obj_key)


    ##  This method will remove an object with object name 'key' from the dictionary
    def remove_object(self, key):
        if self.m_selected_objects_dict.has_key(key):
            del self.m_selected_objects_dict[key]




    ##  This method will create a popup if some error occures
    def error_popup_msg(self, error_name, error_msg):
        title = "Error: " + str(error_name)
        tkMessageBox.showerror(title, error_msg)        


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



