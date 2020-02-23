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

#GUI code
class gui(tk.Frame):
    def dm1control(self):
        fn.dm1()
    def exitBtn(self):
        self.QUIT = tk.Button(self, text='Quit', command = tk.Frame.destroy)
        self.QUIT.grid(column = 1, row = 1)
    def dmOneBtn(self):
        self.btn1 = tk.Button(self, text = 'Drive Mode 1', command = self.dm1control)
        self.btn1.grid(column = 1, row = 2)

    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.grid()
        self.exitBtn()
        self.dmOneBtn()

window = tk.Tk()
window.geometry('350x200')
app = gui(master=window)
app.master.title('Rover Interface')
app.mainloop()


#THE BELOW SHOULD BE MOVED TO FUNCTIONS

#Drive Modes
#DM = 0 is waiting for input about which drive mode to be in
#DM = 1 is 1:1 control - WASD or controller or GUI
#DM = 2 is 7s delay with manual control
#DM = 3 is auto mode
#DM = 4 is for testing things in the code - skips the user input

#DM = 0 selected as default
DM = 0

while(DM==0):
    #Display options
    print("Welcome to Rover 3 \n")
    print("Select Drive Mode")

    #wait for user input

    #parse user input

#========================================================
#Drive mode 1 is manual control (WASD/Controller)
while(DM==1):
    print("we are in DM1")
    #Start control GUI
    #read inputs
    #make a function that sends inputs to arduino
#========================================================
#Drive mode 2 is manual control (WASD/Controller) with delay
while(DM==2):
    print("we are in DM2")
    #start control GUI
    #read inputs
    #make a function that sends inputs to arduino
#========================================================
#Drive mode 3 is automatic mode
while(D==3):
    print("we are in DM3")
    #auto mode
    #read in instructions
    #write function to calculate heading
    #write function to check environment
#========================================================
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