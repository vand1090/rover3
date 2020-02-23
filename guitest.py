import tkinter as tk

class Application(tk.Frame):
    def createWidgets(self):
        self.QUIT = tk.Button(self, text='Quit', command = self.quit)
        self.QUIT.grid()
        
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.grid()
        self.createWidgets()
        
app = Application()
app.master.title('test app')
app.mainloop()


