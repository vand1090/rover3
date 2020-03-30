import serial
import time
#ser = serial.Serial('/dev/ttyACM0', 9600)
import tkinter as tk
import cv2
from picamera import PiCamera
import smbus

import board
import busio
import adafruit_vl53l0x
#from keybinder import KeyBinder
#==========================================================
# Rover 3 Control Code Functions - Raspberry Pi
# Author: Alexander Vanden Bussche
# Spring 2020
#==========================================================
#constants
camera = PiCamera()

restart = True
i2c = busio.I2C(board.SCL, board.SDA)
#tof_sensor = adafruit_vl53l0x.VL53L0X(i2c)
duino_address = 0x04

#==========================================================
#Functions

def standby():
    #use as a pause between commands for debugging.
    #can also be used for drive mode 2
    time.sleep(5)
    #return True
def getRange():
   # print('Range: {}mm'.format(tof_sensor.range))
   time.sleep(1)

def kill():
    #This will be the software kill
    DM = 0
    #stop all movement
    #exit the program
    exit()

def startCam():
    camera.start_preview(fullscreen=False,window=(0,0,600,500)) #xywh

def endCam():
    camera.stop_preview()

def dm1():
    #Drive mode 1 is manual control (WASD/Controller)
    print("Drive Mode 1")
    #Start control GUI
    #read inputs
    #make a function that sends inputs to arduino

    bus = smbus.SMBus(1)
    while(drive=="stop"):
        bus.write_byte(duino_address,6)
        time.sleep(0.5)

    while(drive != "stop"):
        if (drive == "forward"):
            bus.write_byte(duino_address,2)
            time.sleep(0.5)
        elif (drive == "backward"):
            bus.write_byte(address,3)
            time.sleep(0.5)
        elif (drive == "right"):
            bus.write_byte(duino_address,4)
            time.sleep(0.5)
        elif (drive == "left"):
            bus.write_byte(duino_address,5)
            time.sleep(0.5)
        else: #brake
            bus.write_byte(duino_address,6)
            time.sleep(0.5)

    time.sleep(0.1) #wait for 0.5 seconds before checking
        # if a key is pressed

def dm2():
    #Drive mode 2 is manual control (WASD/Controller) with delay
    print("Drive Mode 2")
    #start control GUI
    #read inputs
    #standby()
    #make a function that sends inputs to arduino

def dm3():
    print("Drive Mode 3")
    goal_loc = 1
    count = 1
    drive = True
    while(drive):
        loc = findMe()
        setHeading(loc, goal_loc)
        if(checkObs()):
            avoidObs()
        count = count+1
        if(count == 10):
            drive=False

    #Drive mode 3 is automatic mode
    #auto mode
    #read in instructions
    #write function to calculate heading
    #write function to check environment
#==========================================================
#Autonomous mode functions
def findMe():
    #this function finds the rover in the IPS system
    #Reads via spi bus
    print("Searching")

def checkObs():
    rangeMin = 30; #Rover allowed no closer than this
    rangeRead = 10; #replace this with LIDAR check
    #If there's an obstruction within range, return true
    if (rangeRead <= rangeMin):
        return True
    else:
        return False

def avoidObs():
    #this function tries to move the rover arund an obstacle
    print("avoiding obstacles")

def setHeading(loc, goal):
    print("set heading")

#===========================================================================
#GUI class
class gui(tk.Frame):
    #Define control state commands
        #these are the commands from the buttons on the GUI
        #fn is the rover functions program
        #might change this to open a fresh GUI inside those fn.dm# functions
        #alternative is to change the existing GUI
        #Not sure which option is faster
    def dm1control(self):
        endCam()
        startCam()
        #newWindow = tk.Toplevel(self)
        #dm1_quitBtn = tk.Button(newWindow, text='Quit', command = newWindow.destroy)
        #dm1_quitBtn.pack()
        dm1()

    def dm2control(self):
        endCam()
        startCam()
        dm2()
    def dm3control(self):
        endCam()
        startCam()
        dm3()
    def swKill(self):
        endCam()
        kill()
    #these are the button objects
    def exitBtn(self):
        self.QUIT = tk.Button(self, text='Quit', command = self.swKill)
        self.QUIT.grid(column = 1, row = 1)
    def rstBtn(self):
        self.QUIT = tk.Button(self, text='Restart', command = self.resetSwitch)
        self.QUIT.grid(column = 2, row = 1)
    def dmOneBtn(self):
        self.btn1 = tk.Button(self, text = 'Drive Mode 1', command = self.dm1control)
        self.btn1.grid(column = 1, row = 2)
    def dmTwoBtn(self):
        self.btn2 = tk.Button(self, text = 'Drive Mode 2', command = self.dm2control)
        self.btn2.grid(column = 1, row = 3)
    def dmThreeBtn(self):
        self.btn3 = tk.Button(self, text = 'Drive Mode 3', command = self.dm3control)
        self.btn3.grid(column = 1, row = 4)
    def killSwitch(self):
        self.killBtn = tk.Button(self, text = 'Kill Rover Action', command = self.swKill)
    def resetSwitch(self):
        endCam()
        self.destroy
        self.__init__
    def keyBind(self):
        self.label = tk.Label(self, text="Key Press:  ", width=20)
        self.label.bind("<w>", self.on_w)
        self.label.bind("<a>", self.on_a)
        self.label.bind("<s>", self.on_s)
        self.label.bind("<d>", self.on_d)
        # give keyboard focus to the label by default, and whenever
        # the user clicks on it
        self.label.focus_set()
        self.label.bind("<1>", lambda event: self.label.focus_set())
        self.label.grid(column = 1, row = 5)
    # Set drive = 0 when a key is not pressed

    # while loop for key is not pressed?
    drive = "stop"

    def on_w(self, event):
        self.label.configure(text="Forward");
        drive = "forward"
    def on_a(self, event):
        self.label.configure(text="Left");
        drive = "left"
    def on_s(self, event):
        self.label.configure(text="Backward");
        drive = "backward"
    def on_d(self, event):
        self.label.configure(text="Right");
        drive = "right"
    def on_wasd(newWindow, event):
        newWindow.label.configure(text="last key pressed: " + event.keysym);
    #initialize the frame with those button objects
    def __init__(self, master=None, drive_mode = 0):
        tk.Frame.__init__(self, master)
        #self.vid = MyVideoCapture(video_source)
        self.grid()
        self.exitBtn()
        self.rstBtn()
        self.dmOneBtn()
        self.dmTwoBtn()
        self.dmThreeBtn()
        self.keyBind()

#Reading GPS position. do not use for now
#===================================================================
def readPos():
    try:
        state = ser.readline()
        state_conv = state.decode("utf-8")
        pos = parsePos(state_conv)
        #print(state_conv)
        return pos
    except:
        pass

def parsePos(state_conv):
    posType = state_conv.split(',')
    if (posType[0] == "$GPRMC"):
        # pos = posType[1].split(',')
        # print(pos)
        timeRead = posType[1]
        validBool = posType[2]
        # print(validBool)
        if (validBool) == "V":
            print("No Connection to Satellites at", timeRead)
            # print(timeRead)
            return 0
        elif(validBool == "A"):
            latitude = posType[3]
            lat_dir = posType[4]
            longitude = postype[5]
            long_dir = posType[6]
            roverSpeed = posType[7]
        return [latitude, lat_dir, longitude, long_dir, roverSpeed]

    elif(posType[0] == "$GPGGA"):
        time_ping= posType[1]
        latitude = posType[2]
        lat_dir = posType[3]
        longitude = posType[4]
        long_dir = posType[5]
        return [time_ping, latitude, lat_dir, longitude, long_dir]
#===================================================================