#! /user/bin/env python

import rospy
from geometry_msgs.msg import Pose
from custom_container_tab import RosContainerTab
from sortable_object_batch_class import SortableBatch
from sortable_object_class import SortableObject

import Tkinter as tk
import ttk
import tkMessageBox

import os
import xml.etree.ElementTree as ET

from Tkinter import *
from ttk import *




##  NOTE This class will be used to create a notebook where tabs can be closed via an integrated button
class CustomNotebook(ttk.Notebook):

    __initialized = False
    m_tabs_open = 0
    __sortable_batches = dict()


    def __init__(self, *args, **kwargs):
        if not self.__initialized:
            self.__initialize_custom_style()
            CustomNotebook.__initialized = True

            kwargs["style"] = "CustomNotebook"
            ttk.Notebook.__init__(self, *args, **kwargs)

            self.__active = None
            self.__deploying = False  # This flag is True if the software will run on actual hardware

            self.m_all_open_tabs_dict = dict()
            self.m_all_containers_dict = self.read_all_containers(self.__deploying)  # Dictionary containing all {container : position} pairs
            
            self.m_active_container_dict = dict()  # Dictionary containing all active containers
            self.m_unused_containers_dict = self.m_all_containers_dict  # Dictionary of all unused containers 

            self.bind(sequence="<ButtonPress-1>", func=self.on_close_press, add=True)  # This binds the instance to an event listener, it receives an event, calls a specific callback method and the "add" flag denotes if the specified function replaces the normal behaviour
            self.bind(sequence="<ButtonRelease-1>", func=self.on_close_release)

            self.construct_batch_objects()




    ##  This method will read all containers from an xml and will return a dictionary containing its name and postition
    def read_all_containers(self, is_deploy):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        suffix = ".xml"
        filename = "container_positions"
        
        full_path = os.path.join(dir_path, filename + suffix)

        try: 
            with open(full_path, 'rb') as xml_file:
                tree = ET.parse(xml_file)
        except Exception as e:
                print(e)
                error = "Unable to find file: " + str(filename + str(suffix)) + "\n\nAt: " + str(full_path)
                self.error_popup_msg("Can't find container file", error)
                return None

        root = tree.getroot()
        items = root.getchildren()

        final_dict = {}

        for item in items:
            item_position = item.getchildren()  # TODO: Seems stale, check if this is actually needed

            container_name = item.attrib['name']

            x = item.find('./position/x_pos').text
            y = item.find('./position/y_pos').text
            z = item.find('./position/z_pos').text

            container_position = Pose()
            container_position.position.x = float(x)
            container_position.position.y = float(y)
            container_position.position.z = float(z)

            #  If files will be in the deployable structure using oritentation
            if is_deploy:
                ox = item.find('./orientation/x_orient').text
                oy = item.find('./orientation/y_orient').text
                oz = item.find('./orientation/z_orient').text
                ow = item.find('./orientation/w_orient').text

                container_position.orientation.x = float(ox)
                container_position.orientation.y = float(oy)
                container_position.orientation.z = float(oz)
                container_position.orientation.w = float(ow)

            final_dict[str(container_name)] = container_position 

        return final_dict





    ##  This method will construct all batches and populate them with sortable objects
    def construct_batch_objects(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        suffix = ".xml"
        filename = "object_positions"

        full_path = os.path.join(dir_path,filename+suffix)

        try:
            with open(full_path, 'rb') as xml_file:
                tree = ET.parse(xml_file)
        except Exception as e:
            print(e)
            error = "Unable to find objects in file: " + str(filename + str(suffix)) + "\n\nAt location: " + str(full_path)
            self.error_popup_msg("Can't find object file", error)
            return None

        root = tree.getroot()
        objects = root.getchildren()

        object_types = set([obj_type.text for obj_type in root.iter('type')])  # List of all object types in .xml

        #  Create all batch object instances
        for obj_type in object_types:

            #  Populate batch class 
            batch = SortableBatch(i_object_type=obj_type)

            for item in objects:
                if item.find('./type').text == obj_type:
                    sortable_obj = self.build_sortable_object(item, obj_type, self.__deploying)
                    batch.m_available_objects_dict.update({item.attrib['name'] : sortable_obj})
            CustomNotebook.__sortable_batches.update({str(obj_type) : batch})
  



    ##  This method will take an ET Element and will return a sortable object
    def build_sortable_object(self, et_element, obj_batch_type, is_deploy):
        name = str(et_element.attrib['name'])

        x = et_element.find('./position/x_pos').text
        y = et_element.find('./position/y_pos').text
        z = et_element.find('./position/z_pos').text

        obj_position = Pose()
        obj_position.position.x = float(x)
        obj_position.position.y = float(y)
        obj_position.position.z = float(z)

         #  If files will be in the deployable structure using oritentation
        if is_deploy:
            ox = item.find('./orientation/x_orient').text
            oy = item.find('./orientation/y_orient').text
            oz = item.find('./orientation/z_orient').text
            ow = item.find('./orientation/w_orient').text

            obj_position.orientation.x = float(ox)
            obj_position.orientation.y = float(oy)
            obj_position.orientation.z = float(oz)
            obj_position.orientation.w = float(ow)


        sortable = SortableObject(obj_name=name, obj_position=obj_position, batch_type=obj_batch_type)

        return sortable




    ##  This method will be used to add tabs to the notebook and add said tabs to a list of active tabs
    ##  This will also remove the tab from the unused tab dictionary
    ##  This method will return the tab if successful, or None if the number of available containers is exceeded
    def add_tab(self, parent_note):
        if self.m_all_containers_dict == None:
            return None

        if len(self.m_unused_containers_dict) > 0:  # Make sure that there are consumable keys
            container_name = next(iter(self.m_unused_containers_dict))  # Get the first key from the unused tabs
            container_position = self.m_unused_containers_dict.pop(container_name)  # Remove the item since it is now in use
            
            self.m_active_container_dict.update({container_name: container_position})

            tab = RosContainerTab(parent=parent_note, i_container_name=container_name, 
                                i_container_position=container_position, i_batches=CustomNotebook.__sortable_batches)

            tab.m_container_pose = container_position
            tab.m_batches = self.__sortable_batches  # Add batches to containers

            ttk.Notebook.add(self, tab, text=container_name)  # Add the tab to the notebook
            
            self.m_all_open_tabs_dict.update({container_name:tab})  # Save reference to tab
            self.m_tabs_open += 1
            
            return tab
        return None




    ##  This method will called when the close tab button is pressed, it will receive the event and react
    ##  This method will NOT close the actual tab, but it will record the tab index which should be killed, once the button has been released
    def on_close_press(self, event):
        
        element = self.identify(event.x, event.y)  # Get the name and possition of the tab that spawned the event

        #  Check that tab is up to be closed, and that there are atleast 2 tabs open (not allowed to close all tabs)
        if "close" in element and self.m_tabs_open > 1:
            index = self.index("@%d,%d" % (event.x, event.y))  # Get index of tab to be closed
            self.state(['pressed'])
            self.__active = index  # Keeps track of the tab that we want to close



    
    ##  This method will destroy the tab once the close button has been released
    def on_close_release(self, event):
        
        # Check if the close button really has been pressed
        if not self.instate(['pressed']):
            return

        element = self.identify(event.x, event.y)
        index = self.index("@%d,%d" % (event.x, event.y))  # Get index of tab that has spawned the event

        # Check if element should be closed, and that the current element is the same as the one we tracked
        if "close" in element and index == self.__active:
            rm_tab = self.tab(index)['text']  # Get the key of the tab we are closing
            
            container = self.m_all_open_tabs_dict.get(rm_tab)
            container.clean_destroy()  # Let the container empty all selected objects
            
            container_position = self.m_active_container_dict.pop(rm_tab)

            self.m_unused_containers_dict.update({rm_tab: container_position})

            self.forget(index)
            self.event_generate("<<NotebookTabClose>>")
            

        self.state(["!pressed"])
        self.__active = None  # Clear tracked tab, no longer need it since we removed the tab
        self.m_tabs_open -= 1




    ##  NOTE: This method will be used to set up the style of the notebook and its components
    def __initialize_custom_style(self):
        style = ttk.Style()

        # Define the images which will be used to display the X
        self.images = (
            tk.PhotoImage("img_close", data='''
                R0lGODlhCAAIAMIBAAAAADs7O4+Pj9nZ2Ts7Ozs7Ozs7Ozs7OyH+EUNyZWF0ZWQg
                d2l0aCBHSU1QACH5BAEKAAQALAAAAAAIAAgAAAMVGDBEA0qNJyGw7AmxmuaZhWEU
                5kEJADs=
                '''),
            tk.PhotoImage("img_closeactive", data='''
                R0lGODlhCAAIAMIEAAAAAP/SAP/bNNnZ2cbGxsbGxsbGxsbGxiH5BAEKAAQALAAA
                AAAIAAgAAAMVGDBEA0qNJyGw7AmxmuaZhWEU5kEJADs=
                '''),
            tk.PhotoImage("img_closepressed", data='''
                R0lGODlhCAAIAMIEAAAAAOUqKv9mZtnZ2Ts7Ozs7Ozs7Ozs7OyH+EUNyZWF0ZWQg
                d2l0aCBHSU1QACH5BAEKAAQALAAAAAAIAAgAAAMVGDBEA0qNJyGw7AmxmuaZhWEU
                5kEJADs=
            ''')
        )

        style.element_create("close", "image", "img_close",
                            ("active", "pressed", "!disabled", "img_closepressed"),
                            ("active", "!disabled", "img_closeactive"), border=8, sticky='')
        style.layout("CustomNotebook", [("CustomNotebook.client", {"sticky": "nswe"})])
        style.layout("CustomNotebook.Tab", [
            ("CustomNotebook.tab", {
                "sticky": "nswe", 
                "children": [
                    ("CustomNotebook.padding", {
                        "side": "top", 
                        "sticky": "nswe",
                        "children": [
                            ("CustomNotebook.focus", {
                                "side": "top", 
                                "sticky": "nswe",
                                "children": [
                                    ("CustomNotebook.label", {"side": "left", "sticky": ''}),
                                    ("CustomNotebook.close", {"side": "left", "sticky": ''}),
                                ]
                        })
                    ]
                })
            ]
        })
    ])



    ##  This method will create a popup if some error occures
    def error_popup_msg(self, error_name, error_msg):
        title = "Error: " + str(error_name)
        tkMessageBox.showerror(title, error_msg) 
