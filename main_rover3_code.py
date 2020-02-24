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
window.geometry('350x200+300+300')
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

