from gpiozero import * #uncomment this on the pi
import cv2 #uncomment this on the pi
import rover_functions as fn
import time
import serial
from picamera import PiCamera #uncomment this on the pi
import tkinter as tk
#==========================================================
# Rover 3 Control Code Module A - Raspberry Pi
# Author: Alexander Vanden Bussche
# Spring 2020
#==========================================================

#starts gui window using the class define above
window = tk.Tk()
window.geometry('350x200')
app = fn.gui(master=window)
app.master.title('Rover Interface')
app.mainloop()


#Test code
#testing variables
a =  10
count = 0

#gpio setup

#inputs

#controls

#================================================================================
#Ignore everything below here, this is test code
#while(drive):
    #Read data
    #pos = readPos()
    #r_long = pos[1]
    #print(r_long)
    #standby()
    #if (pos == 0):
       #count = count+1
        #print("Fail Count:",count)
        #standby()

    #This if statement is a plcaeholder for more complicated code
    #if (pos == goalPos):
        #drive = false

    #count if used for shortening the while loop during debugging
    #count = count+1
    #output controls
#video stream class
#Moved to rover functions
#many errors
##class MyVideoCapture:
##    #on inititalize:
##    def __init__(self, video_source=0):
##        #open video source
##        self.vid = cv2.VideoCapture('videotestsrc ! video/x-raw,framerate=14/1 ! videoscale ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
##        if not self.vid.isOpened():
##            raise ValueError("Unable to open video source ", video_source)
##        self.width = self.vid.get(cv2.CAP_PROP_FRAME_WIDTH)
##        self.height = self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT)
##
##     #makes sure the stream ends when you close the object   
##    def __del__(self):
##        if self.vid.isOpened():
##            seld.vid.release()
##        self.window.mainloop()
##        
##    def get_frame(self):
##        if self.vid.isOpened():
##            ret, frame = self.vid.read()
##            if ret:
##                return (ret, cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
##            else:
##                return(ret, None)
##        else:
##            return (ret, None)
#===========================================================================                
#GUI class
###might move to another .py file if I can do it without too many errors
##class gui(tk.Frame):
##    #Define control state commands
##        #these are the commands from the buttons on the GUI
##        #fn is the rover functions program
##        #might change this to open a fresh GUI inside those fn.dm# functions
##        #alternative is to change the existing GUI
##        #Not sure which option is faster
##    def dm1control(self):
##        fn.dm1()
##    def dm2control(self):
##        fn.dm2()
##    def dm3control(self):
##        fn.dm3()
##    #these are the button objects
##    def exitBtn(self):
##        self.QUIT = tk.Button(self, text='Quit', command = tk.Frame.destroy)
##        self.QUIT.grid(column = 1, row = 1)
##    def dmOneBtn(self):
##        self.btn1 = tk.Button(self, text = 'Drive Mode 1', command = self.dm1control)
##        self.btn1.grid(column = 1, row = 2)
##    def dmTwoBtn(self):
##        self.btn1 = tk.Button(self, text = 'Drive Mode 2', command = self.dm1control)
##        self.btn1.grid(column = 1, row = 3)
##    def dmThreeBtn(self):
##        self.btn1 = tk.Button(self, text = 'Drive Mode 3', command = self.dm1control)
##        self.btn1.grid(column = 1, row = 4)
##    #initialize the frame with those button objects
##    def __init__(self, master=None, video_source = 0):
##        tk.Frame.__init__(self, master)
##        self.vid = MyVideoCapture(video_source)
##        self.grid()
##        self.exitBtn()
##        self.dmOneBtn()
##        self.dmTwoBtn()
##        self.dmThreeBtn()


