#! /usr/bin/env python

from __future__ import print_function

#  ROS imports
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import (String, Bool)

import os
import json

#  Graphics imports
import Tkinter as tk  # Works on 2.7
import ttk
from custom_notebook import CustomNotebook
from custom_container_tab import RosContainerTab

#  Helper class imports
from psypdt_dissertation.msg import SortableObjectMessage as SortableObjectMsg
from sortable_object_class import SortableObject
from live_sorting_progress_view_class import LiveViewFrame
from create_new_localised_object import ObjectLocationInputBox

import xml.etree.ElementTree as ET

import tkMessageBox
import tkSimpleDialog

from Tkinter import *
from ttk import *





##  This class will detail how the application window will be constructed
class Application(Frame):

    def __init__(self, master=None):
        #  ROS initialization
        rospy.init_node("main_gui_node")
        self.publisher = rospy.Publisher('ui/sortable_object/sorting/execute', SortableObjectMsg, queue_size=10)  # execute sorting task
        self.completed_sorting_task_sub = rospy.Subscriber('sawyer_ik_sorting/sortable_objects/all/sorted', Bool, callback=self.finished_sorting_callback, queue_size=10)
        
        self.shutdown_ik_pub = rospy.Publisher('sawyer_ik_solver/change_to_state/shudown', Bool, queue_size=10)

        self.add_object_pub = rospy.Publisher('ui/user/is_moving_arm', Bool, queue_size=10)  # Tell IK solver that its not allowed to move

        self.request_final_pos_pub = rospy.Publisher('ui/new_object/state/is_located', Bool, queue_size=10)
        self.final_obj_pos_sub = rospy.Subscriber('position_fetcher/new_object/final_pose', Pose, callback=self.receive_new_object_final_pose_callback, queue_size=10)  # Listen for final object position

        rospy.Rate(10)

        self.is_sorting = False  # This flag will be used to stop the user from adding items while the sorting task is executing
        self.is_creating_container = False  #  This flag will change depending on what type of additon the user is making (object or container)

        #  Graphics initialization
        Frame.__init__(self, master)
        self.master.minsize(830,500)

        #  Create Notebook 
        self.notebook = CustomNotebook()
        
        #  Create Tabs and add 
        self.create_tab(self.notebook)
        self.notebook.pack(expand=1, fill="both")

        self.pack()
        self.createWidgets()

        master.title('SAWYER SORTING TASK GUI')



    ##  This method will send a SortableObjectMsg to the ik solver
    def send_object_to_sort(self):
        #  Prevent user from trying to spam the sort button
        if self.is_sorting == True:
            return

        self.is_sorting = True  # Set flag to indicate that sorting is in progress

        is_add_msg = Bool(data=False)
        self.add_object_pub.publish(is_add_msg)

        if self.notebook.m_all_open_tabs_dict.values() <= 0:
            error_name = "No Containers!"
            error_msg = "There are no containers! Try adding some with the <Add Container> button"
            self.error_popup_msg(error_name, error_msg)
            self.is_sorting = False  # Reset flag since sorting failed
            return

        #  For every tab, get m_selected_objects_dict
        for tab in self.notebook.m_all_open_tabs_dict.values():

            for item in tab.m_selected_objects_dict.values():  # Get all selected SortableObjects
                msg = item.to_sortableObjectMessage()
                self.publisher.publish(msg)
                
        self.top_lvl_window = tk.Toplevel(self.master)
        self.top_lvl_window.title("Live Sorting Progress")
        self.top_lvl_window.minsize(550,400)
        self.live_sort_window = LiveViewFrame(self.top_lvl_window, i_all_containers=self.notebook.m_all_open_tabs_dict.keys(), i_selected_objects=self.get_selected_objects())



    ##  Callback to allow user to add containers and objects again, after the sorting task has completed
    def finished_sorting_callback(self, state):
        if state.data == True:
            self.is_sorting = False



    ##  Create widgets which are not specific to tabs
    def createWidgets(self):
        style = ttk.Style()  # Create style for buttons
        style.configure("WR.TButton", foreground="white", background="red", width=20, height=20)

        self.run = ttk.Button(self, text="RUN SORTING TASK", style="WR.TButton", command= lambda: self.send_object_to_sort())
        self.run.pack(side='left', ipadx=10, padx=30)

        self.locate_object_button = ttk.Button(self, text="Create new object", command=self.add_new_object_pose)
        self.locate_object_button.pack(side='right', ipadx=10, padx=30)
        
        self.add_container_button = ttk.Button(self, text="Add Container", command= lambda: self.create_tab(self.notebook))
        self.add_container_button.pack(side='left', ipadx=10, padx=30)

        self.create_new_container_button = ttk.Button(self, text="Create new container", command=self.add_new_container_pose)
        self.create_new_container_button.pack(side='right', ipadx=10, padx=30)



    ##  This method will be used to add tabs to the notebook
    def create_tab(self, note):
        tab = note.add_tab(parent_note=note)

        if tab != None:
            #  Create a grid with dimensions 10x10
            for i in range(10):
                tab.rowconfigure(i, weight=1)
                tab.columnconfigure(i, weight=1)



    ##  This method allows users to add a new object position 
    def add_new_object_pose(self):
        #  User can't add objects while sorting is executing
        if self.is_sorting:
            return

        is_add_msg = Bool(data=True)
        self.add_object_pub.publish(is_add_msg)

        prompt = tkMessageBox.askokcancel('Create New object', 'Please move the robot arm over an object you wish to add.\nOnce you have manually moved the arm over the object, please click \'OK\'')
        is_add_msg = Bool(data=False)

        if prompt == True:
            is_add_msg = Bool(data=True)  # Send this to get the final positon  
            self.add_object_pub.publish(is_add_msg)
            
            self.is_creating_container = False  # Reset this flag to false since the user is adding an object
            
            is_done_msg = Bool(data=True)
            self.request_final_pos_pub.publish(is_done_msg)

        else:
            is_add_msg = Bool(data=False)
            self.add_object_pub.publish(is_add_msg)



    ##  This method will tell the IK solver that the user is adding a new position 
    def add_new_container_pose(self):
        #  User can't add container while robot is sorting
        if self.is_sorting:
            return

        is_add_msg = Bool(data=True)
        self.add_object_pub.publish(is_add_msg)

        prompt = tkMessageBox.askokcancel('Create New Container', 'Please move the robot arm over a Container you wish to add.\nOnce you have manually moved the arm over the object, please click \'OK\'')
        is_add_msg = Bool(data=False)

        if prompt == True:
            is_add_msg = Bool(data=True)  # Send this to get the final positon  
            self.add_object_pub.publish(is_add_msg)
            
            self.is_creating_container = True  # Set this flag to True since the user is adding a new container
            
            is_done_msg = Bool(data=True)
            self.request_final_pos_pub.publish(is_done_msg)

        else:
            is_add_msg = Bool(data=False)
            self.add_object_pub.publish(is_add_msg)



    ##  This is a callback which will get the pose of a new object the user wants to add
    def receive_new_object_final_pose_callback(self, pose):
        #  Ask the user to provide a name and type for the object
        self.top_lvl_prompt_window = tk.Toplevel(self.master)
        self.top_lvl_prompt_window.title("Create New Object")
        self.top_lvl_prompt_window.minsize(300,80)

        #  If the user is adding a container choose tehe appropriate input box
        if self.is_creating_container:
            self.new_object_prompt = ObjectLocationInputBox(self.top_lvl_prompt_window, pose, self.notebook, is_object=False)
        else:
            self.new_object_prompt = ObjectLocationInputBox(self.top_lvl_prompt_window, pose, self.notebook)



    ##  This method returns a list of all selected object names
    def get_selected_objects(self):
        selected_obj_list = []
        
        for container in self.notebook.m_all_open_tabs_dict.values():
            for item in container.m_selected_objects_dict.keys():
                selected_obj_list.append(item)
        return selected_obj_list


    ##  This method will create a popup if some error occures
    def error_popup_msg(self, error_name, error_msg):
        title = "Error: " + str(error_name)
        tkMessageBox.showerror(title, error_msg)

    

    ##  This method will shutdown the current and the ik_solver node
    def shutdown_nodes(self):
        signal = Bool(True)
        self.shutdown_ik_pub.publish(signal)
        rospy.signal_shutdown("User closed main ui window")





root = Tk()  # The window which will contain all components
root.geometry('850x600')  # Default size of window 

app = Application(master=root)


def handle_close():
    app.shutdown_nodes()
    root.destroy()


root.protocol('WM_DELETE_WINDOW', handle_close)
app.mainloop()



