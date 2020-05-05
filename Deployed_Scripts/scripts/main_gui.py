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
from psypdt_dissertation.msg import PoseGrippMessage
from sortable_object_class import SortableObject
from live_sorting_progress_view_class import LiveViewFrame
from create_new_localised_object import ObjectLocationInputBox 
from waypoints_class import Waypoints

import xml.etree.ElementTree as ET
import threading

import tkMessageBox
import tkSimpleDialog

from Tkinter import *
from ttk import *





##  This class will detail how the application window will be constructed
class Application(Frame):

    def __init__(self, master=None):
        #  ROS initialization
        self.__to_sort_list_lock = threading.Lock()
        self.objects_to_send_list=[]  # A list containing all objects wich must be send to the ik solver
 
        rospy.init_node("main_gui_node")
        self.gui_pub_sort_execute_item = rospy.Publisher('ui/sortable_object/sorting/execute', SortableObjectMsg, queue_size=20)  # execute sorting task
        # self.gui_sub_completed_item = rospy.Subscriber('sawyer_ik_sorting/sortable_objects/object/sorted', SortableObjectMsg, callback=self.send_next_item_callback, queue_size=10)  # Listen to this topic to know when an item was sorted so we can send the next one
        self.gui_sub_failed_item = rospy.Subscriber('sawyer_ik_solver/sorting/has_failed', SortableObjectMsg, callback=self.send_item_after_failure_callback, queue_size=10)  # Listen for failed items, if one is detected then send the next item

        self.completed_sorting_task_sub = rospy.Subscriber('sawyer_ik_sorting/sortable_objects/all/sorted', Bool, callback=self.finished_sorting_callback, queue_size=10)
        self.gui_pub_abort_sorting = rospy.Publisher('gui/user/has_aborted', Bool, queue_size=1)
        
        self.shutdown_ik_pub = rospy.Publisher('sawyer_ik_solver/change_to_state/shudown', Bool, queue_size=10)
        self.gui_pub_user_is_moving_arm = rospy.Publisher('ui/user/is_moving_arm', Bool, queue_size=10)  # Tell IK solver that its not allowed to move

        self.request_final_pos_pub = rospy.Publisher('ui/new_object/state/is_located', Bool, queue_size=10)
        self.final_obj_pos_sub = rospy.Subscriber('position_fetcher/new_object/final_pose', PoseGrippMessage, callback=self.receive_new_object_final_pose_callback, queue_size=10)  # Listen for final object position

        rospy.Rate(10)

        self.is_sorting = False  # This flag will be used to stop the user from adding items while the sorting task is executing
        self.is_creating_container = False  #  This flag will change depending on what type of additon the user is making (object or container)

        self.waypoints = None


        #  Graphics initialization
        Frame.__init__(self, master)
        self.master.minsize(830,520)

        #  Create Notebook 
        self.notebook = CustomNotebook()
        
        #  Create Tabs and add 
        self.create_tab(self.notebook)
        self.notebook.pack(expand=1, fill="both")

        self.pack()
        self.createWidgets()

        master.title('SAWYER SORTING TASK GUI')




    ##  Create widgets which are not specific to tabs
    def createWidgets(self):
        style = ttk.Style()  # Create style for buttons
        style.configure("WR.TButton", foreground="white", background="green", width=20, height=20)

        #  Pack the start and stop button into one frame
        self.sorting_command_frame = Frame(self)

        #  Start button
        self.run = ttk.Button(self.sorting_command_frame, text="RUN SORTING TASK", style="WR.TButton", command=self.send_object_to_sort)
        self.run.pack(side='top', ipadx=10, padx=10)

        self.sorting_command_frame.pack(side='left', ipadx=10)


        # Add frame for the waypoint functionality
        self.waypoint_frame = Frame(self)

         #  Pressing this button allow the user to set waypoints
        self.waypoint_set_button = tk.Button(self.waypoint_frame, text="Create Path", bg='orange', width=15, command=self.create_waypoints_callback)
        self.waypoint_set_button.pack(side='bottom', ipadx=10, padx=8)

        # Button to run the waypoint
        self.waypoint_play_button = tk.Button(self.waypoint_frame, text="Execute Path", bg="red", fg="white", width=15, command=self.run_waypoints_callback)
        self.waypoint_play_button.pack(side='bottom', ipadx=10, padx=8)
        
        self.waypoint_frame.pack(side="right", ipadx=10)


        #  Add container, use tk. button to do coloring
        # self.add_container_button = tk.Button(self, bg='#3adee0', text="Add Container", command= lambda: self.create_tab(self.notebook))
        self.add_container_button = ttk.Button(self, text="Activate Existing Container", width=18, command= lambda: self.create_tab(self.notebook))
        self.add_container_button.pack(side='left', ipadx=10, padx=20)

        self.registration_frame = Frame(self)

        #  Create Objects & Containers
        self.locate_object_button = ttk.Button(self.registration_frame, text="Register New Object", width=17, command=self.add_new_object_pose)
        self.locate_object_button.pack(side='top', ipadx=10, padx=20)

        self.create_new_container_button = ttk.Button(self.registration_frame, text="Register New Container", width=17, command=self.add_new_container_pose)
        self.create_new_container_button.pack(side='bottom', ipadx=10, padx=20)

        self.registration_frame.pack(side="right", ipadx=10)




    ##  This method will send a SortableObjectMsg to the ik solver
    def send_object_to_sort(self):
        #  Prevent user from trying to spam the sort button
        if self.is_sorting == True:
            return

        self.is_sorting = True  # Set flag to indicate that sorting is in progress

        is_add_msg = Bool(data=False)
        self.gui_pub_user_is_moving_arm.publish(is_add_msg)  #  Tell IK solver that it's allowed to move the arm

        if self.notebook.m_all_open_tabs_dict.values() <= 0:
            self.is_sorting = False  # Reset flag since sorting failed

            error_name = "No Containers!"
            error_msg = "There are no containers! Try adding some with the <Add Container> button"
            self.error_popup_msg(error_name, error_msg)
            return

        #  For every tab, get m_selected_objects_dict
        for tab in self.notebook.m_all_open_tabs_dict.values():
            #  TODO: Maybe look into sorting the items by object_name so that they get sorted in the order that the user sees them on the main gui
            #  Generate list of items to send
            for item in tab.m_selected_objects_dict.values():  # Get all selected SortableObjects
                msg = item.to_sortableObjectMessage()
                self.objects_to_send_list.append(msg)

        #  Check that there are items to send  
        if len(self.objects_to_send_list) > 0:
            # Create live view for user
            top_lvl_window = tk.Toplevel(self.master)
            top_lvl_window.title("Live Sorting Progress")
            top_lvl_window.minsize(550,400)
            live_sort_window = LiveViewFrame(top_lvl_window, i_all_containers=self.notebook.m_all_open_tabs_dict.keys(), i_selected_objects=self.get_selected_objects(), waypoints=self.waypoints)
        else:
            self.is_sorting = False 
            return

        #  Send all messages
        for msg in self.objects_to_send_list:
            self.gui_pub_sort_execute_item.publish(msg)
        
        #  Clear the list
        del self.objects_to_send_list[:]
        
                



    ##  This method is a callback that will be used to send a new sortable object once the ik solver has completed sorting a single object
    def send_next_item_callback(self, data):
        #  Make sure that the sorting task is still active (hasn't been stopped by user)
        if self.is_sorting:

            self.__to_sort_list_lock.acquire()  # Enter critical section

            if len(self.objects_to_send_list) > 0:
                next_message = self.objects_to_send_list.pop()
                self.__to_sort_list_lock.release()  # End of critical section

                self.gui_pub_sort_execute_item.publish(next_message)
                


    
    ##  This callback is invoked if an object was not sorted, to prevent blocking we call this to send the next item
    def send_item_after_failure_callback(self, data):
        #  Make sure we are in sorting mode
        if self.is_sorting:
        
            self.__to_sort_list_lock.acquire()  # Entering critical section
            
            if len(self.objects_to_send_list) > 0:
                next_message = self.objects_to_send_list.pop()
                self.__to_sort_list_lock.release()  # Exit critical section

                self.gui_pub_sort_execute_item.publish(next_message)
                




    ##  Callback to allow user to add containers and objects again, after the sorting task has completed
    def finished_sorting_callback(self, state):
        if state.data == True:
            self.is_sorting = False
            



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
        self.gui_pub_user_is_moving_arm.publish(is_add_msg)
        new_obj_message = 'Please move the robot arm over an object you wish to add.\n\nOnce you have manually moved the arm over the object, ' \
        'close the gripper until the object is firmly grasped. \n\nOnce you have done this please click \'OK\''

        prompt = tkMessageBox.askokcancel('Register New Object', new_obj_message)
        is_add_msg = Bool(data=False)

        #  User wants to add new object
        if prompt == True:
            is_add_msg = Bool(data=True)    
            self.gui_pub_user_is_moving_arm.publish(is_add_msg)
            
            self.is_creating_container = False  # Reset this flag to false since the user is adding an object
            
            is_done_msg = Bool(data=True)
            self.request_final_pos_pub.publish(is_done_msg)  # Send this to get the final positon

        else:
            is_add_msg = Bool(data=False)
            self.gui_pub_user_is_moving_arm.publish(is_add_msg)



    ##  This method will tell the IK solver that the user is adding a new position 
    def add_new_container_pose(self):
        #  User can't add container while robot is sorting
        if self.is_sorting:
            return

        is_add_msg = Bool(data=True)
        self.gui_pub_user_is_moving_arm.publish(is_add_msg)

        new_container_message = 'Please move the robot arm over a Container you wish to add.\n\nOnce you have manually moved the arm over the object, please click \'OK\''
        prompt = tkMessageBox.askokcancel('Register New Container', new_container_message)
        is_add_msg = Bool(data=False)

        if prompt == True:
            is_add_msg = Bool(data=True)  
            self.gui_pub_user_is_moving_arm.publish(is_add_msg)
            
            self.is_creating_container = True  # Set this flag to True since the user is adding a new container
            
            is_done_msg = Bool(data=True)
            self.request_final_pos_pub.publish(is_done_msg)  # Retreive final position from robot

        else:
            is_add_msg = Bool(data=False)
            self.gui_pub_user_is_moving_arm.publish(is_add_msg)



    ##  This is a callback which will get the pose of a new object the user wants to add
    def receive_new_object_final_pose_callback(self, pose):
        #  Ask the user to provide a name and type for the object
        self.top_lvl_prompt_window = tk.Toplevel(self.master)
        self.top_lvl_prompt_window.title("Register New Object")
        self.top_lvl_prompt_window.minsize(300,80)

        #  If the user is adding a container choose tehe appropriate input box
        if self.is_creating_container:
            self.new_object_prompt = ObjectLocationInputBox(self.top_lvl_prompt_window, pose, self.notebook, is_object=False)
        else:
            self.new_object_prompt = ObjectLocationInputBox(self.top_lvl_prompt_window, pose, self.notebook)
        
        #  Reset robot arm to default position
        is_add_msg = Bool(data=False)
        self.gui_pub_user_is_moving_arm.publish(is_add_msg)



    ##  This method returns a list of all selected object names
    def get_selected_objects(self):
        selected_obj_list = []
        
        for container in self.notebook.m_all_open_tabs_dict.values():
            for item in container.m_selected_objects_dict.keys():
                selected_obj_list.append(item)
        return selected_obj_list
        
        

    ##  Use the waypoints functionality
    def create_waypoints_callback(self):
        if not self.is_sorting:
            name = "Waypoint Instructions"
            instructions = "Waypoints are used to create a path the robot will follow.\n\nMove the arm into a position, click the large gray wheel to remember the point.\
                    \n\nYou may need to save multiple points to construct a path for the robot.\n\nOnce you have constructed a path, press the Rethink button to finish and save.\
                    \n\nAlternatively, press the circle to reset the path to nothing. \n\nPress 'OK' to start."
            
            self.show_important_warning_msg(name, instructions) 
            self.waypoint_play_button.configure(bg='green')  # Signal to the user that the button can be used
            
            self.waypoints = Waypoints(speed=0.28, timeout=3)
            self.waypoints.record()



    ##  Callback to run the waypoints
    def run_waypoints_callback(self):
        if not self.is_sorting  and self.waypoints != None and len(self.waypoints.waypoints) > 0:
            self.waypoints.playback()
        else:
            error_name = "No Path!"
            description = "There is no path which can be executed!\n\nTry creating on with the 'Create Path' button."
            self.error_popup_msg(error_name, description)




    ##  This method will create a popup if some error occures
    def error_popup_msg(self, error_name, error_msg):
        title = "Error: " + str(error_name)
        tkMessageBox.showerror(title, error_msg)




    ##  This method will display a warning popup
    def show_important_warning_msg(self, warning_name, warning_msg):
        title = "Warning: " + str(warning_name)
        tkMessageBox.showwarning(title, warning_msg)
        

    

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



