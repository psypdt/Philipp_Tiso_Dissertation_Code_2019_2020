#! /usr/bin/env python


import rospy
from std_msgs.msg import Bool
from intera_examples.msg import SortableObjectMessage

import threading

import Tkinter as tk
import ttk
import tkMessageBox



class LiveViewFrame(ttk.Frame):

    def __init__(self, parent=None, i_all_containers=None, i_selected_objects=None):
        self.__item_lock = threading.Lock()  # Lock to make sure no two callbacks manipulate the list at the same time

        #  Create Subscribe to topic, cant create node since class runs in main_gui process
        self.sub_sorted = rospy.Subscriber('sawyer_ik_sorting/sortable_objects/object/sorted', SortableObjectMessage, callback=self.update_container_status_callback, queue_size=10)  # Topic where sorted object:container pairs get sent to
        self.sub_sorting_error = rospy.Subscriber('sawyer_ik_solver/sorting/has_failed', SortableObjectMessage, callback=self.handle_failed_sort_callback, queue_size=10)
        self.completed_sorting_pub = rospy.Publisher('sawyer_ik_sorting/sortable_objects/all/sorted', Bool, queue_size=10)
        self.sub_task_aborted = rospy.Subscriber('gui/user/has_aborted', Bool, callback=self.handle_aborted_sorting_callback, queue_size=10)  # GUI will notify subscribers if user has stopped sorting task


        ttk.Frame.__init__(self, parent)
        self.m_active_containers = i_all_containers
        self.m_selected_objects = i_selected_objects

        self.m_container_item_dict = dict()  # Dictionary containing a list of items for each container
        self.setup_widgets()



    ##  This method will setup all widgets in the live view
    def setup_widgets(self):
        self.create_container_widgets()
        self.create_objects_selected_widget()
        self.pack()



    ##  Create widget displaying all selected objects
    ##  TODO: Create scrollable frame for this
    def create_objects_selected_widget(self):
        self.obj_selected_var = tk.StringVar()
        self.obj_selected_var.set(str(self.m_selected_objects))

        selected_label = tk.Label(self, text="Objects to sort:", font=("Helvetica", 10))
        self.object_list_label = ttk.Label(self, textvar=self.obj_selected_var, font=("Helvetica", 10), wraplength=250, justify='left')
        
        self.object_list_label.pack(side='bottom')
        selected_label.pack(side='bottom')




    ##  This method will initialize all container widgets
    def create_container_widgets(self):
        self.canvas = tk.Canvas(self)

        self.container_TreeView = ttk.Treeview(self.canvas)
        self.container_TreeView["columns"] = ["Container", "Item", "Status"]
        self.container_TreeView["show"] = "headings"

        #  Create headers for treeview
        self.container_TreeView.heading("Container", text="Container")    
        self.container_TreeView.heading("Item", text="Item")
        self.container_TreeView.heading("Status", text="Status")  # Did the sorting fail or succeed 

        #  Define highlighting for successful sorts
        self.container_TreeView.tag_configure('successful_sort', background='green', foreground='white')

        #  Define some highliting for identifying failed & aborted sorts
        self.container_TreeView.tag_configure('failed_sort', background='red')  # Anything with this tag will be highlited in red
        self.container_TreeView.tag_configure('aborted_sort', background='orange')

        #  Create horizontal scrollbar
        treeXscroll = ttk.Scrollbar(self.canvas, orient="horizontal")
        treeXscroll.configure(command=self.container_TreeView.xview)
        self.container_TreeView.configure(xscrollcommand=treeXscroll.set)

        #  Create vertical scrollbar
        treeYscroll = ttk.Scrollbar(self.canvas, orient="vertical")
        treeYscroll.configure(command=self.container_TreeView.yview)
        self.container_TreeView.configure(yscrollcommand=treeYscroll.set)

        #  Pack and size the scroll bars to fit the respective axis
        treeXscroll.pack(side='bottom',fill='x')
        treeYscroll.pack(side='right', fill='y')

        tree_title_label = tk.Label(self.canvas, text="Sorted Container : Item pairs", font=("Helvetica", 12))

        tree_title_label.pack(side='top')
        self.container_TreeView.pack()
        self.canvas.pack()



    ##  Callback method that will update the text in the container widgets
    def update_container_status_callback(self, data):
        self.__item_lock.acquire()  # Get lock for critical section

        #  Remove item from list & update the list
        if len(self.m_selected_objects) >= 1 and data.object_name in self.m_selected_objects:
            self.m_selected_objects.remove(data.object_name)

            self.__item_lock.release()  # End critical

            self.obj_selected_var.set(self.m_selected_objects)
            
            if len(self.m_selected_objects) <= 0:
                self.obj_selected_var.set("All objects have been sorted")
                is_complete = Bool(data=True)
                self.completed_sorting_pub.publish(is_complete)

            self.container_TreeView.insert("", tk.END, values=(str(data.container_name).capitalize(), str(data.object_name).capitalize(), "SORTED"), tags=('successful_sort',))
        else:
            self.__item_lock.release()  # Release to prevent blocking
        



    ##  Handle cases where an object was not sorted correctly
    def handle_failed_sort_callback(self, data):
        self.__item_lock.acquire() # Get lock for critical section

        if len(self.m_selected_objects) >= 1 and data.object_name in self.m_selected_objects:  # Check if object is in list
            self.m_selected_objects.remove(data.object_name)
            
            self.__item_lock.release()  # End of critical section 

            self.obj_selected_var.set(self.m_selected_objects)  # Set the label string for the objects which have not been sorted
            
            if len(self.m_selected_objects) <= 0:
                self.obj_selected_var.set("All objects have been sorted")
                is_complete = Bool(data=True)
                self.completed_sorting_pub.publish(is_complete)

            self.container_TreeView.insert("", tk.END, values=(str(data.container_name).capitalize(), str(data.object_name).capitalize(), "FAILED"), tags=('failed_sort',))
        else:
            self.__item_lock.release()  # Release lock to prevent blocking



    
    ##  This callback is invoked if the user has canceled the entire sorting task
    def handle_aborted_sorting_callback(self, state):
        if state.data == False:
            return

        self.__item_lock.acquire()

        #  While items can be consumed
        while len(self.m_selected_objects) >= 1:
            item = self.m_selected_objects.pop()

            self.container_TreeView.insert("", tk.END, values=(str("None"), str(item).capitalize(), "ABORTED"), tags=('aborted_sort'))
        
        self.__item_lock.release()  # End of critical section
        self.obj_selected_var.set(self.m_selected_objects)  # Update the label containing current items to sort

        #  Notify main_gui that task is complete
        is_complete = Bool(data=True)
        self.completed_sorting_pub.publish(is_complete)

        # tkMessageBox.showwarning("Halting Task!", "Halting Robot.\n\nItems may still be enqueued! \n\nStay clear of the robot for atleast 20 seconds.")


