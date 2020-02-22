#! /usr/bin/env python


import rospy
from intera_examples.msg import SortableObjectMessage
import Tkinter as tk
import ttk


existing_containers_dict = dict()  # Dictionary containing container name and StringVar detailing all sorted objects
selected_objects_label = []



class LiveViewFrame(ttk.Frame):

    def __init__(self, parent=None, i_all_containers=None, i_selected_objects=None):
        #  Create Subscribe to topic, cant create node since class runs in main_gui process
        self.sub_sorted = rospy.Subscriber('data/sorting/sorted_objects', SortableObjectMessage, queue_size=10)  # Topic where sorted object:container pairs get sent to

        ttk.Frame.__init__(self, parent)
        self.m_active_containers = i_all_containers
        self.m_selected_objects = i_selected_objects
        
        self.setup_widgets()



    def setup_widgets(self):
        self.create_container_widgets()
        self.object_list_label = tk.Label(self, text="Objects to sort: " + str(self.m_selected_objects))
        self.object_list_label.pack(side='left', ipadx=10, pady=10)
        
        self.pack()



    ##  This method will initialize all container widgets
    def create_container_widgets(self):
        self.canvas = tk.Canvas(self)

        self.container_TreeView = ttk.Treeview(self.canvas)
        self.container_TreeView["columns"] = self.m_active_containers  # Columns are the containers 
        self.container_TreeView["show"] = "headings"

        for c in self.m_active_containers:
            self.container_TreeView.heading(c, text=c)

        

        #  Create scroll bar to scroll horizontally and vertically
        treeXscroll = ttk.Scrollbar(self.canvas, orient="horizontal")
        treeXscroll.configure(command=self.container_TreeView.xview)
        self.container_TreeView.configure(xscrollcommand=treeXscroll.set)

        self.container_TreeView.pack()
        treeXscroll.pack(side='bottom', ipadx=30, pady=30)

        self.canvas.pack()



    ##  Callback method that will update the text in the container widgets
    def update_container_status_callback():
        pass
