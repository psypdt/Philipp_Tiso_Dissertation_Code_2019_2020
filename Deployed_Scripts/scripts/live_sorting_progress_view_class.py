#! /usr/bin/env python


import rospy
from std_msgs.msg import Bool
from psypdt_dissertation.msg import SortableObjectMessage
from waypoints_class import Waypoints

import threading

import Tkinter as tk
import ttk
import tkMessageBox



class LiveViewFrame(ttk.Frame):

    def __init__(self, parent=None, i_all_containers=None, i_selected_objects=None, waypoints=None):

        #  Create Subscribe to topic, cant create node since class runs in main_gui process
        self.sub_sorted = rospy.Subscriber('sawyer_ik_sorting/sortable_objects/object/sorted', SortableObjectMessage, callback=self.update_container_status_callback, queue_size=10)  # Topic where sorted object:container pairs get sent to
        self.completed_sorting_pub = rospy.Publisher('sawyer_ik_sorting/sortable_objects/all/sorted', Bool, queue_size=10)

        self.waypoints = waypoints  # Waypoints will be executed last

        ttk.Frame.__init__(self, parent)

        self.m_active_containers = i_all_containers
        self.m_selected_objects = i_selected_objects  # List: Names of all selected objects

        self.m_container_item_dict = dict()  # Dictionary containing a list of items for each container
        self.setup_widgets()



    ##  This method will setup all widgets in the live view
    def setup_widgets(self):
        self.create_container_widgets()
        self.create_objects_selected_widget()
        self.pack()



    ##  Create widget displaying all selected objects
    def create_objects_selected_widget(self):
        self.obj_selected_var = tk.StringVar()
        self.obj_selected_var.set(str(self.m_selected_objects))
        
        self.textbox_frame = tk.Frame(self)
        self.textbox_frame.pack(side='bottom')


        #  Objects to sort title
        items_to_sort_label = tk.Label(self.textbox_frame, text="Objects to sort:", font=("Helvetica", 10, 'bold'))
        # self.object_list_label = ttk.Label(self, textvar=self.obj_selected_var, font=("Helvetica", 10), wraplength=250, justify='left')
        items_to_sort_label.pack(side='top')

        
        #  Create textbox 
        #  Create horizontal scroll
        self.horizontal_textbox_scroll = tk.Scrollbar(self.textbox_frame, orient="horizontal")
        self.horizontal_textbox_scroll.pack(side="bottom", fill="x")

        #  Create vertical scroll
        self.vertical_textbox_scroll = tk.Scrollbar(self.textbox_frame, orient="vertical")
        self.vertical_textbox_scroll.pack(side="right", fill="y")

        #  Create textbox for user to see selected objects
        self.selected_objects_textbox = tk.Text(self.textbox_frame, wrap="none", width=25, height=10, font=("Helvetica", 8, 'bold'),
                                                yscrollcommand=self.vertical_textbox_scroll.set, xscrollcommand=self.horizontal_textbox_scroll.set)
        self.selected_objects_textbox.config(state="disabled")
        self.selected_objects_textbox.pack(side="bottom")


        #  Configure scroll commands
        self.horizontal_textbox_scroll.config(command=self.selected_objects_textbox.xview)
        self.vertical_textbox_scroll.config(command=self.selected_objects_textbox.yview)
        
        self.update_textbox_objects_label()  # Display all current items


        #  Waypoint text
        waypoint_text = "Custom_Path" if self.waypoints != None and len(self.waypoints.waypoints) > 0 else "None"
        self.waypoint_label = tk.Label(self, text="Waypoints", font=("Helvetica", 10, 'bold'))
        self.waypoint_list_label = tk.Label(self, text=waypoint_text, font=("Helvetica", 10))
        
        self.waypoint_list_label.pack(side="bottom")
        self.waypoint_label.pack(side="bottom")






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
        self.container_TreeView.tag_configure('in_progress_sort', background='orange')
        # self.container_TreeView.tag_configure('aborted_sort', background='orange')

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

        tree_title_label = tk.Label(self.canvas, text="Sorting Progress", font=("Helvetica", 12, 'bold'))

        tree_title_label.pack(side='top')
        self.container_TreeView.pack()
        self.canvas.pack()


        #  Add the first item in the sorting list, mark it as in progress
        self.container_TreeView.insert("", tk.END, values=("...", str(self.m_selected_objects[0]).capitalize(), "IN PROGRESS"), tags=('in_progress_sort',))



    ##  Callback method that will update the text in the container widgets when an object has been either sorted, or failed to sort
    def update_container_status_callback(self, data):

        #  Remove item from list & update the list
        if len(self.m_selected_objects) >= 1 and data.object_name in self.m_selected_objects:
            self.m_selected_objects.remove(data.object_name)

            # self.obj_selected_var.set(self.m_selected_objects)
            self.update_textbox_objects_label()
            
            if len(self.m_selected_objects) <= 0:
                # self.obj_selected_var.set("All objects have been sorted")
                self.update_textbox_objects_label(is_done=True)
                is_complete = Bool(data=True)
                self.completed_sorting_pub.publish(is_complete)

                self.sub_sorted.unregister()

            #  Check that there is an in progress item to remove
            if len(self.container_TreeView.get_children()) > 0:
                last_item = self.container_TreeView.get_children()[-1]  # Get last item
                self.container_TreeView.delete(last_item)  # Delete last item
                

            #  Highlight the object appropriatly depending on if it was sorted or not
            if data.successful_sort == True:
                self.container_TreeView.insert("", tk.END, values=(str(data.container_name).capitalize(), str(data.object_name).capitalize(), "SORTED"), tags=('successful_sort',))
                
            elif data.successful_sort == False:
                self.container_TreeView.insert("", tk.END, values=(str(data.container_name).capitalize(), str(data.object_name).capitalize(), "FAILED"), tags=('failed_sort',))
         
            #  Check if we can mark the next object as in progress
            if len(self.m_selected_objects) > 0:
                self.container_TreeView.insert("", tk.END, values=("...", str(self.m_selected_objects[0]).capitalize(), "IN PROGRESS"), tags=('in_progress_sort',))


            #  Execute waypoint
            if self.waypoints != None and len(self.m_selected_objects) <= 0 and len(self.waypoints.waypoints) > 0:
                self.container_TreeView.insert("", tk.END, values=("...", "Custom Path (Waypoints)", "IN PROGRESS"), tags=('in_progress_sort',))
                self.waypoints.playback()  # Execute waypoints

                last_item = self.container_TreeView.get_children()[-1]  # Get last item
                self.container_TreeView.delete(last_item)  # Delete last item
                
                self.container_TreeView.insert("", tk.END, values=("Custom Destination", "Waypoint Object", "SORTED"), tags=('successful_sort',))
                self.waypoint_list_label.config(text="Waypoints executed")



    ##  This method will update the text widget displaying all selected items
    def update_textbox_objects_label(self, is_done = False):
        self.selected_objects_textbox.config(state="normal")  # Set state to normal so we can delete old data & insert new data
        
        if is_done:
            new_text = "All objects have been sorted!"
        else:
            new_text = self.get_sorting_queue() 
        
        self.selected_objects_textbox.delete('1.0', tk.END)  # Remove old content from start (1.0) to the end
        self.selected_objects_textbox.insert('1.0', new_text)
        self.selected_objects_textbox.config(state="disabled")  # Prevent user from editing text
        self.selected_objects_textbox.pack(side="bottom")



    ##  This method will return a string of all items which aren't sorted 
    def get_sorting_queue(self):
        final_str = ""

        for item in self.m_selected_objects:
            final_str += str(item) + "\n\n"
        return final_str



    ##  Unregister all subscribers 
    def clean_destroy(self):
        self.completed_sorting_pub.unregister()
        self.sub_sorting_error.unregister()
        self.sub_task_aborted.unregister()
