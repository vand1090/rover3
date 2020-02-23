from gpiozero import *
import cv2
from rover_functions import *
import time
import serial
from picamera import PiCamera
#==========================================================
# Rover 3 Control Code Module A - Raspberry Pi
# Author: Alexander Vanden Bussche
# Spring 2020
#==========================================================
# Variables & Constants
#Drive Modes
#DM0 is waiting for input about which drive mode to be in
#DM1 is 1:1 control - WASD
#DM2 is waypoint nav
#DM3 is 7s delay (asteroid mode)
while(DM0):
    #t-
    ake inputs



drive = True
#goalPos is a placeholder for more complicated code
goalPos =1



#testing variables
a =  10
count = 0

#gpio setup

#inputs

#controls

while(drive):
    #Read data
    pos = readPos()
    r_long = pos[1]
    print(r_long)
    standby()
    if (pos == 0):
        count = count+1
        print("Fail Count:",count)
        standby()

    #This if statement is a plcaeholder for more complicated code
    if (pos == goalPos):
        drive = false

    #count if used for shortening the while loop during debugging
    #count = count+1
    #output controls