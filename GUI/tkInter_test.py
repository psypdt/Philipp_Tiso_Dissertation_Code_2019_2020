#! /usr/bin/env python

from __future__ import print_function

# ROS imports
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import json

#  Graphics imports
import Tkinter as tk  #  Works on 2.7
import ttk
from customNotebook import CustomNotebook

#  Helper class imports
from intera_examples.msg import SortableObjectMessage as SortableObjectMsg
from sorting_object_class import SortableObject
import xml.etree.ElementTree as ET

from Tkinter import *
from ttk import *


#  This method will read all objects from an xml and will return a dictionary containing said objects (including name and postition)
def read_all_objects():
    tree = ET.parse('object_positions.xml')
    root = tree.getroot()

    items = root.getchildren()

    final_dict = {}

    for item in items:
        item_position = item.getchildren()

        name = item.attrib

        x = item.find('./position/x_pos').text
        y = item.find('./position/y_pos').text
        z = item.find('./position/z_pos').text

        start = Pose()
        start.position.x = float(x)
        start.position.y = float(y)
        start.position.z = float(z)

        obj = SortableObject(obj_name=str(name), obj_start=start)

        final_dict[str(name)] = start

    return final_dict




#  This class will detail how the application window will be constructed
class Application(Frame):
    
    m_tabs = 0  #  The number of tabs currently open

    def say_hi(self, label):
        print("Check state is" % label)
        # self.var += 1
        # label.configure(text=self.var)


    def update_selected_objects(self, name, state):

        if state == 1:
            print("State is 1, add %s to list" % name)
        else:
            print("State is 0, remove %s from list" % name)


    #  This method will send a SortableObjectMsg to the ik solver
    def send_object_pos(self, obj_position, container_position):

        send = SortableObjectMsg('first', obj_position, container_position)
        self.publisher.publish(send)


    ##  Create widgets which are not specific to tabs
    #   TODO: Replace the QUIT button with a RUN button, talks to ik solver
    def createWidgets(self):

        final_pos = Pose()

        #  Final Positions
        final_pos.position.x = 0.704020578925
        final_pos.position.y = 0.6890
        final_pos.position.z = 0.455

        #  Final Orientation
        final_pos.orientation.x = 0.0
        final_pos.orientation.y = 0.0
        final_pos.orientation.z = 0.0
        final_pos.orientation.w = 1.0

        style = ttk.Style()  # Create style for buttons
        style.configure("WR.TButton", foreground="white", background="red", width=20, height=20)

        self.run = ttk.Button(self, text="RUN", command= lambda: self.send_object_pos(final_pos, final_pos))
        self.run.pack(side='left', ipadx=10, padx=30)

        self.QUIT = ttk.Button(self, text="Quit", style="WR.TButton", command=self.quit)
        self.QUIT.pack(side='left', ipadx=10, padx=30)
        
        self.add_container_button = ttk.Button(self, text="Add Container", command= lambda: self.create_tab(self.note))
        self.add_container_button.pack(side='right', ipadx=10, padx=30)



    ##  This method will be used to add tabs to the notebook
    def create_tab(self, note):

        # Limited to 5 containers for now
        if note.m_tabs_open == 5:
            return

        tab = ttk.Frame(note)

        #  Create a grid with dimensions 10x10
        for i in range(10):
            tab.rowconfigure(i, weight=1)
            tab.columnconfigure(i, weight=1)
            

        tabName = "Container " + str(note.m_tabs_open)
        note.add(tab, text=tabName)

        note.m_tabs_open = note.m_tabs_open + 1  # Increment tab counter
        
        self.instantiate_tab(tab)



    ##  This method takes in a tab instance and will add the relevant widgets to it
    ##  NOTE: Dont use pack() if grid is used in the same method
    def instantiate_tab(self, tab):
        
        # self.count_label = ttk.Label(tab, text="Test label")
        self.object1 = IntVar()
        self.object2 = IntVar()
        self.object3 = IntVar()

        Checkbutton(tab, text="obj1", variable=self.object1, onvalue=1, offvalue=0, command=lambda: self.update_selected_objects("obj1", self.object1.get())).grid(row=1, column=9)  # The command is what will be called when the box is pressed
        Checkbutton(tab, text="obj2", variable=self.object2, onvalue=1, offvalue=0, command=lambda: self.update_selected_objects("obj2", self.object2.get())).grid(row=2, column=9)
        Checkbutton(tab, text="obj3", variable=self.object3, onvalue=1, offvalue=0, command=lambda: self.update_selected_objects("obj3", self.object3.get())).grid(row=3, column=9)

        # self.test_button = ttk.Button(tab, text="Add Container", command= lambda: self.create_tab(self.note)).grid(row=3, column=7)
        # self.test_button.pack({"side":"right"})


    def __init__(self, master=None):
        #  ros initialization
        rospy.init_node("main_gui_node")
        self.publisher = rospy.Publisher('move_to_dest/goal', SortableObjectMsg, queue_size=10)


        #  Graphics initialization
        Frame.__init__(self, master)

        #  Create Notebook 
        self.note = CustomNotebook()
        
        #  Create Tabs and add 
        self.create_tab(self.note)
        # self.homeTab = ttk.Frame(self.note)
        # self.note.add(self.homeTab, text="Container 0")

        self.note.pack(expand=1, fill="both")

        self.m_tabs = 1

        # self.instantiate_tab(self.homeTab)
        self.pack()
        self.createWidgets()
        master.title('ROS TEST UI')
        

root = Tk()  #  The window which will contain all components
root.geometry('750x500')  #  Default size of window 

app = Application(master=root)


app.mainloop()
root.destroy()