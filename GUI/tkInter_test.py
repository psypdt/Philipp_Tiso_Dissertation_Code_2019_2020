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

from Tkinter import *
from ttk import *



class Application(Frame):
    
    m_tabs = 0  #  The number of tabs currently open

    def say_hi(self, label):
        print("hi there, everyone! %s" % (self.var))
        self.var += 1
        label.configure(text=self.var)


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

        Checkbutton(tab, text="obj1", variable=self.object1, onvalue=1, offvalue=0, command=None).grid(row=1, column=9)  # The command is what will be called when the box is pressed
        Checkbutton(tab, text="obj2", variable=self.object2, onvalue=1, offvalue=0, command=None).grid(row=2, column=9)
        Checkbutton(tab, text="obj3", variable=self.object3, onvalue=1, offvalue=0, command=None).grid(row=3, column=9)


        # self.count_label.pack()

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