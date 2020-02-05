import Tkinter as tk  # Works on 2.7
import ttk
from customNotebook import CustomNotebook

from Tkinter import *
from ttk import *



class Application(Frame):
    
    m_tabs = 0  # The number of tabs currently open

    def say_hi(self, label):
        print("hi there, everyone! %s" % (self.var))
        self.var += 1
        label.configure(text=self.var)


    ##  Create widgets which are not specific to tabs
    def createWidgets(self):
        style = ttk.Style()  # Create style for buttons
        style.configure("WR.TButton", foreground="white", background="red", width=20, height=20)

        self.QUIT = ttk.Button(self, text="Quit", style="WR.TButton", command=self.quit)
        self.QUIT.pack(side='left', ipadx=10, padx=30)
        
        self.test_button = ttk.Button(self, text="Add Container", command= lambda: self.create_tab(self.note))
        self.test_button.pack(side='right', ipadx=10, padx=30)



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
        
        self.count_label = ttk.Label(tab, text="Test label").grid(row=5, column=7)
        # self.count_label.pack()

        # self.test_button = ttk.Button(tab, text="Add Container", command= lambda: self.create_tab(self.note)).grid(row=3, column=7)
        # self.test_button.pack({"side":"right"})




    def __init__(self, master=None):
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
        



root = Tk()  # The window which will contain all components
root.geometry('750x500')  # Default size of window 

app = Application(master=root)


app.mainloop()
root.destroy()