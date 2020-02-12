import Tkinter as tk
import ttk

from Tkinter import *
from ttk import *




##  SOURCE: https://stackoverflow.com/questions/39458337/is-there-a-way-to-add-close-buttons-to-tabs-in-tkinter-ttk-notebook/55888727#55888727



##  NOTE This class will be used to create a notebook where tabs can be closed via an integrated button
class CustomNotebook(ttk.Notebook):

    __initialized = False
    m_tabs_open = 0

    def __init__(self, *args, **kwargs):
        if not self.__initialized:
            self.__initialize_custom_style()
            CustomNotebook.__initialized = True

            kwargs["style"] = "CustomNotebook"
            ttk.Notebook.__init__(self, *args, **kwargs)

            self.__active = None
            self.__tabs_open = 0

            self.bind(sequence="<ButtonPress-1>", func=self.on_close_press, add=True)  # This binds the instance to an event listener, it receives an event, calls a specific callback method and the "add" flag denotes if the specified function replaces the normal behaviour
            self.bind(sequence="<ButtonRelease-1>", func=self.on_close_release)



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
            self.forget(index)
            self.event_generate("<<NotebookTabClose>>")

        self.state(["!pressed"])
        self.__active = None  # Clear tracked tab, no longer need it since we removed the tab
        self.m_tabs_open = self.m_tabs_open - 1




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
