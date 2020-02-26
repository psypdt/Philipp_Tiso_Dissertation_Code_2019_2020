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
from intera_examples.msg import SortableObjectMessage as SortableObjectMsg
from sortable_object_class import SortableObject
from object_position_updating_service import LiveViewFrame
import ros_xml_manipulation as RXM

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
        self.publisher = rospy.Publisher('/ui/sort_command/execute', SortableObjectMsg, queue_size=10)
        
        self.shutdown_ik_pub = rospy.Publisher('rsdk_flex_ik_service_client/shudown', Bool, queue_size=10)

        self.add_object_pub = rospy.Publisher('ui/define_object_location/', Bool, queue_size=10)  # Tell IK solver that its not allowed to move

        self.request_final_pos_pub = rospy.Publisher('/ui/new_object/state/done', Bool, queue_size=10)
        self.final_obj_pos_sub = rospy.Subscriber('/live_pose_node/object/final_pose', Pose, callback=self.receive_new_object_final_pose, queue_size=10)  # Listen for final object position

        rospy.Rate(10)

        #  Graphics initialization
        Frame.__init__(self, master)

        #  Create Notebook 
        self.notebook = CustomNotebook()
        
        #  Create Tabs and add 
        self.create_tab(self.notebook)
        self.notebook.pack(expand=1, fill="both")

        self.pack()
        self.createWidgets()

        master.title('SAWYER SORTING TASK UI')



    ##  This method will send a SortableObjectMsg to the ik solver
    def send_object_pos(self):
        is_add_msg = Bool(data=False)
        self.add_object_pub.publish(is_add_msg)


        if self.notebook.m_all_open_tabs_dict.values() <= 0:
            error_name = "No Containers!"
            error_msg = "There are no containers! Try adding some with the <Add Container> button"
            self.error_popup_msg(error_name, error_msg)
            return

        #  For every tab, get m_selected_objects_dict
        for tab in self.notebook.m_all_open_tabs_dict.values():

            for item in tab.m_selected_objects_dict.values():  # Get all selected SortableObjects
                msg = item.to_sortableObjectMessage()
                self.publisher.publish(msg)
                
        self.top_lvl_window = tk.Toplevel(self.master)
        self.top_lvl_window.title("Live View")
        self.top_lvl_window.minsize(550,400)
        self.live_sort_window = LiveViewFrame(self.top_lvl_window, i_all_containers=self.notebook.m_all_open_tabs_dict.keys(), i_selected_objects=self.get_selected_objects())



    ##  Create widgets which are not specific to tabs
    ##  TODO: Replace the QUIT button with a RUN button, talks to ik solver
    def createWidgets(self):

        style = ttk.Style()  # Create style for buttons
        style.configure("WR.TButton", foreground="white", background="red", width=20, height=20)

        self.run = ttk.Button(self, text="RUN SORTING TASK", style="WR.TButton", command= lambda: self.send_object_pos())
        self.run.pack(side='left', ipadx=10, padx=30)

        self.locate_object_button = ttk.Button(self, text="Create new object", command=self.add_new_object_pose)
        self.locate_object_button.pack(side='right', ipadx=10, padx=30)
        
        self.add_container_button = ttk.Button(self, text="Add Container", command= lambda: self.create_tab(self.notebook))
        self.add_container_button.pack(side='right', ipadx=10, padx=30)



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
        is_add_msg = Bool(data=True)
        self.add_object_pub.publish(is_add_msg)

        prompt = tkMessageBox.askokcancel('Locate new object', 'Once you have manually moved the arm over the object, please click \'OK\'')
        is_add_msg = Bool(data=False)

        if prompt == True:
            is_add_msg = Bool(data=True)  # Send this to get the final positon  
            self.add_object_pub.publish(is_add_msg)
            
            is_done_msg = Bool(data=True)
            self.request_final_pos_pub.publish(is_done_msg)

        else:
            is_add_msg = Bool(data=False)
            self.add_object_pub.publish(is_add_msg)



    ##  This is a callback 
    def receive_new_object_final_pose(self, pose):
        #  Ask the user to provide a name and type for the object
        # name_prompt = tkSimpleDialog.askstring('Object Name', 'What should this object be refered to as?')

        # if name_prompt != None:

        #     type_prompt = tkSimpleDialog.askstring('Object type', 'What class does this object belong? (Cubes, Screws, Balls, etc)')
        
        # dialogue = ObjectInputBox(self)

        # name = str(dialogue.name_entry.get()).lower()
        # obj_type = str(dialogue.type_entry.get()).lower()

        # self.write_new_object_xml(name, obj_type, pose)


        self.top_lvl_prompt_window = tk.Toplevel(self.master)
        self.top_lvl_prompt_window.title("Create New Object")
        self.top_lvl_prompt_window.minsize(300,80)
        self.new_object_prompt = ObjectLocationInputBox(self.top_lvl_prompt_window, pose)



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







class ObjectLocationInputBox(ttk.Frame):

    def __init__(self, parent=None, obj_pose=None):
        ttk.Frame.__init__(self, parent)
        
        self.new_pose = obj_pose

        self.setup_widgets()
        self.pack()


    def setup_widgets(self):
        
        self.name_label = ttk.Label(self, text="Object Name (Hammer, Apple, etc.)")
        self.name_entry = Entry(self)

        # self.name_label.pack(side="left", anchor="w")
        # self.name_entry.pack(side="right", anchor="e")


        self.type_label = ttk.Label(self, text="Object Type (Tool, Fruit, etc.)")
        self.type_entry = Entry(self)

        # self.type_label.pack(side="left", anchor="w")
        # self.type_entry.pack(side="right", anchor="e")
        

        self.name_label.grid(row=0, column=0, sticky="w", pady=2)
        self.type_label.grid(row=1, column=0, sticky="w", pady=2)

        self.name_entry.grid(row=0, column=1, pady=2)
        self.type_entry.grid(row=1, column=1, pady=2)


        self.done_button = Button(self, text="Done", command=self.write_new_object_xml)
        self.cancel_button = Button(self, text="Cancel")

        self.done_button.grid(row=3, column=0)
        self.cancel_button.grid(row=3, column=1)



    ##  This method will write a new object into an xml file that stores all existing objects
    def write_new_object_xml(self):
        new_name = str(self.name_entry.get())
        new_type = str(self.type_entry.get())

        if not new_name or not new_type or new_name == None or new_type == None:
            return
        
        RXM.append_to_xml_file(filename="object_positions", name=new_name, obj_type=new_type, pose=self.new_pose)

        self.master.destroy()




root = Tk()  # The window which will contain all components
root.geometry('750x500')  # Default size of window 

app = Application(master=root)


def handle_close():
    app.shutdown_nodes()
    root.destroy()


root.protocol('WM_DELETE_WINDOW', handle_close)
app.mainloop()






