#! /usr/bin/env python


import Tkinter as tk
import ttk





##  This class will allow us to have a dropdown menue 
class ToggledFrame(tk.Frame):

    def __init__(self, parent, collection_name="N/A", *args, **options):
        tk.Frame.__init__(self, parent, *args, **options)

        self.show = tk.IntVar()  # Save toggle state
        self.show.set(0)  # Initially collapsed 

        #  Create frame to place dropdown into
        self.title_frame = ttk.Frame(self)
        self.title_frame.pack(fill="x", expand=1)

        #  Setup title of collection, fills entire frame
        batch_text = str(collection_name) + "Batch"
        ttk.Label(self.title_frame, text=batch_text).pack(side="left", fill="x", expand=1)

        #  Create button which will toggle the view
        self.toggle_button = ttk.Checkbutton(self.title_frame, width=2, text='+', command=self.toggle_state, variable=self.show, style='Toolbutton')
        self.toggle_button.pack(side="left")

        self.sub_frame = tk.Frame(self, relief="sunken", borderwidth=1)


    ##  This method will display the contents of the current toggle frame
    def toggle_state(self):
        if bool(self.show.get()):
            self.sub_frame.pack(fill="x", expand=1)
            self.toggle_button.configure(text='-')
        else:
            self.sub_frame.forget()
            self.toggle_button.configure(text='+')





if __name__ == "__main__":
    root = tk.Tk()

    t = ToggledFrame(root, collection_name='Rotate', relief="raised", borderwidth=1)
    t.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")

    ttk.Label(t.sub_frame, text='Rotation [deg]:').pack(side="left", fill="x", expand=1)
    ttk.Entry(t.sub_frame).pack(side="left")

    t2 = ToggledFrame(root, collection_name='Resize', relief="raised", borderwidth=1)
    t2.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")

    for i in range(10):
        ttk.Label(t2.sub_frame, text='Test' + str(i)).pack()

    t3 = ToggledFrame(root, collection_name='Fooo', relief="raised", borderwidth=1)
    t3.pack(fill="x", expand=1, pady=2, padx=2, anchor="n")

    for i in range(10):
        tk.Checkbutton(t3.sub_frame, text='Bar' + str(i)).pack()

    root.mainloop()