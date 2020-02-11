#! /usr/bin/env python

import Tkinter as tk
import ttk


#  This class will be used to display the list of objects
class RosTab(ttk.Frame):
    
    def __init__(self, parent=None, i_container=None):
        ttk.Frame.__init__(self, parent)

        self.m_container_name = i_container
        self.m_selected_objects_dict = {}

        self.setup_widgets()



    #  This method will call all other methods which are responsible for setting up the tabs gui
    def setup_widgets(self):
        self.setup_object_selection()


    #  This method will set up the object selection widgets for each tab
    def setup_object_selection(self):
        object1 = IntVar()
        object2 = IntVar()
        object3 = IntVar()

        Checkbutton(tab, text="obj1", variable=self.object1, onvalue=1, offvalue=0, command=lambda: self.update_selected_objects("obj1", object1.get())).grid(row=1, column=9)  # The command is what will be called when the box is pressed
        Checkbutton(tab, text="obj2", variable=self.object2, onvalue=1, offvalue=0, command=lambda: self.update_selected_objects("obj2", object2.get())).grid(row=2, column=9)
        Checkbutton(tab, text="obj3", variable=self.object3, onvalue=1, offvalue=0, command=lambda: self.update_selected_objects("obj3", object3.get())).grid(row=3, column=9)



    #  This method will take care of what objects are added and removed from the selected_objects dictionary
    def update_selected_objects(self, name, state):

        print("Tab is %s" % self.m_container_name)
        if state == 1:
            print("State is 1, add %s to list" % name)
            self.add_object(name, 1)
        else:
            print("State is 0, remove %s from list" % name)
            self.remove_object(name)
        
        print("Tab %s has dict %s" % (self.m_container_name, self.m_selected_objects_dict))



    #  This method can be used to add an object to the m_selected_objects_dict dictionary
    def add_object(self, name, value):
        self.m_selected_objects_dict[str(name)] = value


    #  This method will remove an object with object name 'key' from the dictionary
    def remove_object(self, key):
        if self.m_selected_objects_dict.has_key(key):
            del self.m_selected_objects_dict[key]
