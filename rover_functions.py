import serial
import time
import math
ser = serial.Serial('/dev/ttyACM0', 9600)

import tkinter as tk
import cv2
from picamera import PiCamera
import smbus

import board
import busio
import adafruit_vl53l0x
import adafruit_lsm303dlh_mag

from marvelmind import MarvelmindHedge
from debouncer import Debouncer
#from keybinder import KeyBinder
#==========================================================
# Rover 3 Control Code Functions - Raspberry Pi
# Author: Alexander Vanden Bussche
# Spring 2020
#==========================================================
#in itialize sensors and buses
camera = PiCamera()
drive = 6
restart = True
i2c = busio.I2C(board.SCL, board.SDA)
bus = smbus.SMBus(1)
tof_sensor = adafruit_vl53l0x.VL53L0X(i2c)
compass = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
duino_address = 0x04
hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) # create MarvelmindHedge thread
hedge.start() # start thread
LARGE_FONT= ("Verdana", 12)

#Direct modes constants
w = 0
s = 0
shift = 0
dm2delay = 2

#semi-auto vars, do not mess with
currentPos = []
currentX = 0
currentY = 0
currentZ = 0
drive = True

#==========================================================
#Functions

def standby():
    #use as a pause between commands for debugging.
    #can also be used for drive mode 2
    time.sleep(1)
    #return True

def getRange():
   print('Range: {}mm'.format(tof_sensor.range))
   #time.sleep(1)
   return tof_sensor.range

def driving(mode):
    #sends packet to arduino
    bus.write_byte(duino_address,mode)
    time.sleep(0.5)
    #requests to get packet back from arduino
    readNum()
    #pause, may change later
    #time.sleep(0.5)

def readNum():
    #asks the arduino to send a packet
    num = bus.read_byte(duino_address)
    print("RPI Received", num)
    return num

def kill():
    #This will be the software kill
    #driving(0) is kill switch
    driving(0)
    #DM = 0
    #stop all movement
    #exit the program
    exit()

def startCam():
    #in this code the rover will open a camera preview
    camera.start_preview(fullscreen=False,window=(0,0,200,200)) #xywh

def endCam():
    #this closes the camerai preview
    camera.stop_preview()

def dm1():
    #Drive mode 1 is manual control (WASD/Controller)
    print("Drive Mode 1")
    #reading controls is done thru the GUI
    #make sure it intializes to stopped,send 0 packet
    driving(0)

def dm2():
    #Drive mode 2 is manual control (WASD/Controller) with delay
    print("Drive Mode 2")
    #start control GUI
    #read inputs
    #standby()
    #make a function that sends inputs to arduino
    driving(0)

def dm3(goalX, goalY):
    driving(0)
    print("Drive Mode 3")

    while(drive):
        findMe()
        if(goalX == currentX and goalY == currentY):
            print('Success')
            drive = False
            driving(0)
            break
        
        if(currentX)

        deltaX = abs(currentX-goalX)
        deltaY = abs(currentY-goalY)

        hypotenuse = calcDist(deltaX, deltaY)
        driving(1)


        # goal_loc = 1
    # count = 1
    # drive = True
    # while(drive):
    #     loc = findMe()
    #     setHeading(loc, goal_loc)
    #     if(checkObs() == 2):
    #         avoidObs(2)
    #     count = count+1
    #     if(count == 10):
    #         drive=False

    #Drive mode 3 is automatic mode
    #auto mode
    #read in instructions
    #write function to calculate heading
    #write function to check environment
#==========================================================
#Autonomous mode functions
def findMe():
    #this function finds the rover in the IPS system
    hedge.print_position()
    currentPos = hedge.position()
    currentX = currentPos[0]
    currentY = currentPos[1]
    currentZ = currentPos[2]

def checkObs():
    rangeMin = 30 #Rover allowed no closer than this (mm)
    rangeAvoid = 100
    rangeRead = getRange() #Reads the LIDAR
    #If there's an obstruction within range, return true
    if (rangeRead <= rangeMin):
        print("WARNING: Too Close")
        avoidObs(2)
        return 2
    elif(rangeRead <= rangeAvoid):
        print("Obstacle Detected. Begin Reroute Procedure")
        avoidObs(1)
        return 1
    else:
        print("No obstacles")
        return 0

def avoidObs(severity):
    #this function tries to move the rover arund an obstacle
    print("avoiding obstacles")

    if(severity == 2):
        driving(0) #stop the rover, then back it up
        #back & right
        driving(0) #change to correct value - look up
        standby()
        checkObs()

    elif(severity == 1):
        #slow down, start turning
        driving(4) # forward left
        standby()
        checkObs()

    else:
        print("no obstacles to avoid")

# calc total distance to target
def calcDist(deltaX, deltaY):
    return math.sqrt(math.pow(deltaX, 2) + math.pow(deltaY,2))

# compass code
def vector_2_degrees(x, y):
    angle = degrees(atan2(y, x))
    if angle < 0:
        angle += 360
    return angle


def get_heading(_sensor):
    magnet_x, magnet_y, _ = _sensor.magnetic
    return vector_2_degrees(magnet_x, magnet_y)
#===========================================================================
#GUI class
class gui(tk.Tk):

    #initialize base frame as an array of 4 possible windows
    #main menu, DM1, Dm2, Dm3
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand = True)
        container.grid_rowconfigure(1, weight=1)
        container.grid_columnconfigure(1, weight=1)
        #self.grid()
        self.windows = {}

        #initializes each window so it can be shown later
        for F in (mainMenu, dm1Page, dm2Page, dm3Page):
            frame = F(container, self)
            self.windows[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")
        self.show_new_window(mainMenu)

    #start by showing main menu
    def show_new_window(self, cont):
        frame = self.windows[cont]
        frame.tkraise()


#this is the first thing that will open
class mainMenu(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self,parent)
        label = tk.Label(self, text="Select Drive Mode", font=LARGE_FONT)
        label.grid(column=1, row=1)

        #show all the buttons, generated below
        self.grid()
        self.exitBtn()
        self.rstBtn()
        #self.startBtn()
        self.dmOneBtn(controller)
        self.dmTwoBtn(controller)
        self.dmThreeBtn(controller)

    def exitBtn(self): #kill the program
        self.QUIT = tk.Button(self, text='Quit', command = self.swKill)
        self.QUIT.grid(column = 1, row = 2)
    def rstBtn(self): # restarts main menu
        self.QUIT = tk.Button(self, text='Restart', command = self.resetSwitch)
        self.QUIT.grid(column = 2, row = 2)

    def dmOneBtn(self, controller): #bring up drive mode 1
        self.btn1 = tk.Button(self, text = 'Drive Mode 1', command =lambda: controller.show_new_window(dm1Page))
        self.btn1.grid(column = 1, row = 3)

    def dmTwoBtn(self, controller): #bring up drive mode 2
        self.btn2 = tk.Button(self, text = 'Drive Mode 2', command =lambda: controller.show_new_window(dm2Page))
        self.btn2.grid(column = 1, row = 4)

    def dmThreeBtn(self, controller): #bring up drive mode 3
        self.btn3 = tk.Button(self, text = 'Drive Mode 3', command =lambda: controller.show_new_window(dm3Page))
        self.btn3.grid(column = 1, row = 5)

    def swKill(self): # stops all rover action
        endCam()
        kill()

    def killSwitch(self):
        self.killBtn = tk.Button(self, text = 'Kill Rover Action', command = self.swKill)


    def resetSwitch(self): #function for reset button
        endCam()
        self.destroy
        self.__init__


#here begins the gui for drive mode 1. Implements a keylistener
class dm1Page(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Drive Mode 1", font=LARGE_FONT)
        label.grid(column=1, row=1)

        self.grid()
        self.exitBtn()
        self.mainMenuBtn(controller)
        self.startBtn() #controls wont work until this is pressed
        #startCam()
        #dm1()
        #move this to control modes:
        self.keyBind()

    def exitBtn(self):
        self.QUIT = tk.Button(self, text='Quit', command = self.swKill)
        self.QUIT.grid(column = 1, row = 2)

    def mainMenuBtn(self, controller):
        self.btnMM = tk.Button(self, text = 'Main Menu', command =lambda: controller.show_new_window(mainMenu))
        self.btnMM.grid(column = 2, row = 2)

    def swKill(self):
        endCam()
        kill()

    def startBtn(self): #makes stat button
        self.start = tk.Button(self, text = 'START', command = self.startCommand)
        self.start.grid(column = 1, row = 3)

    def startCommand(self):
        startCam()
        dm1()

    def keyBind(self):
        self.label = tk.Label(self, text="Key Press:  ", width=20)

        self.shift = Debouncer(self.on_shift,self.off_shift)
        self.w = Debouncer(self.on_w,self.off_w)
        self.a = Debouncer(self.on_a,self.off_a)
        self.s = Debouncer(self.on_s,self.off_s)
        self.d = Debouncer(self.on_d,self.off_d)
        self.x = Debouncer(self.on_x,self.off_x)

        self.label.bind("<Shift_L>", self.shift.released)
        self.label.bind("<w>", self.w.pressed)
        self.label.bind("<a>", self.a.pressed)
        self.label.bind("<s>", self.s.pressed)
        self.label.bind("<d>", self.d.pressed)
        self.label.bind("<x>", self.x.pressed)

        self.label.bind("<KeyRelease-Shift_L>", self.shift.released)
        self.label.bind("<KeyRelease-w>", self.w.released)
        self.label.bind("<KeyRelease-a>", self.a.released)
        self.label.bind("<KeyRelease-s>", self.s.released)
        self.label.bind("<KeyRelease-d>", self.d.released)
        self.label.bind("<KeyRelease-x>", self.x.released)
        # give keyboard focus to the label by default, and whenever
        # the user clicks on it
        self.label.focus_set()
        self.label.bind("<1>", lambda event: self.label.focus_set())
        self.label.grid(column = 1, row = 5)

    def on_shift(self,event):
        global shift
        shift = 1
    def off_shift(self, event):
        global shift
        shift = 0
    def on_w(self, event):
        self.label.configure(text="Forward")
        if dm1:
            driving(1)
        elif dm2:
            time.sleep(dm2delay)
            driving(1)
    def off_w(self, event):
        print("off w")
        driving(0)
    def on_a(self, event):
        self.label.configure(text="Left")
        if dm1:
            driving(3)
        elif dm2:
            time.sleep(2)
            driving(3)
    def off_a(self, event):
        driving(15)

    def on_s(self, event):
        self.label.configure(text="Backward")
        if dm1:
            driving(2)
        elif dm2:
            time.sleep(2)
            driving(2)
    def off_s(self, event):
        driving(15)

    def on_d(self, event):
        self.label.configure(text="Right")
        if dm1:
            driving(4)
        elif dm2:
            time.sleep(2)
            driving(4)
    def off_d(self, event):
        driving(15)

    def on_x(self, event):
        driving(0)
    def off_x(self, event):
        global shift
        global w
        shift = 0
        w = 0

    def on_wasd(self, event):
        self.label.configure(text="last key pressed: " + event.keysym)


class dm2Page(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Drive Mode 2", font=LARGE_FONT)
        label.grid(column=1, row=1)

        self.grid()
        self.exitBtn()
        self.mainMenuBtn(controller)

        #startCam()
        #dm2()

    def exitBtn(self):
        self.QUIT = tk.Button(self, text='Quit', command = self.swKill)
        self.QUIT.grid(column = 1, row = 2)

    def mainMenuBtn(self, controller):
        self.btnMM = tk.Button(self, text = 'Main Menu', command =lambda: controller.show_new_window(mainMenu))
        self.btnMM.grid(column = 2, row = 2)

    def swKill(self):
        endCam()
        kill()


class dm3Page(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Drive Mode 3", font=LARGE_FONT)
        label.grid(column=1, row=1)
        #populate the window
        self.grid()
        self.exitBtn()
        self.mainMenuBtn(controller)
        self.startBtn()
        self.inputXLabel()
        self.inputXGoal(controller)
        self.inputYLabel()
        self.inputYGoal(controller)

        #startCam()
        #dm3()

    #buttons, labels, inputs
    def exitBtn(self): #kills the rover
        self.QUIT = tk.Button(self, text='Quit', command = self.swKill)
        self.QUIT.grid(column = 1, row = 2)

    def mainMenuBtn(self, controller): #main menu
        self.btnMM = tk.Button(self, text = 'Main Menu', command =lambda: controller.show_new_window(mainMenu))
        self.btnMM.grid(column = 2, row = 2)
    #get x goal
    def inputXGoal(self, controller):
        self.inputXField = tk.Entry(self)
        self.inputYField.grid(column = 2, row = 2)
    def inputXLabel(self):
        self.inXlabel = tk.Label(self, text="Input X Goal", font=LARGE_FONT)
        self.inXlabel.grid(column=1, row = 2)
    #get y goal
    def inputYGoal(self, controller):
        self.inputYField = tk.Entry(self)
        self.inputYField.grid(column = 2, row = 3)
    def inputYLabel(self):
        self.inYlabel = tk.Label(self, text="Input Y Goal", font=LARGE_FONT)
        self.inYlabel.grid(column=1, row = 3)


    def swKill(self):
        endCam()
        kill()
    def startBtn(self): #makes stat button
        self.start = tk.Button(self, text = 'START', command = self.startCommand)
        self.start.grid(column = 1, row = 3)
    # this is when auto mode starts
    def startCommand(self):
        goalX = self.inputXField.get()
        goalY = self.inputYField.get()
        startCam()
        dm3(goalX, goalY)

#Reading GPS position. do not use for now
#===================================================================
# def readPos():
#     try:
#         state = ser.readline()
#         state_conv = state.decode("utf-8")
#         pos = parsePos(state_conv)
#         #print(state_conv)
#         return pos
#     except:
#         pass

# def parsePos(state_conv):
#     posType = state_conv.split(',')
#     if (posType[0] == "$GPRMC"):
#         # pos = posType[1].split(',')
#         # print(pos)
#         timeRead = posType[1]
#         validBool = posType[2]
#         # print(validBool)
#         if (validBool) == "V":
#             print("No Connection to Satellites at", timeRead)
#             # print(timeRead)
#             return 0
#         elif(validBool == "A"):
#             latitude = posType[3]
#             lat_dir = posType[4]
#             longitude = postype[5]
#             long_dir = posType[6]
#             roverSpeed = posType[7]
#         return [latitude, lat_dir, longitude, long_dir, roverSpeed]

#     elif(posType[0] == "$GPGGA"):
#         time_ping= posType[1]
#         latitude = posType[2]
#         lat_dir = posType[3]
#         longitude = posType[4]
#         long_dir = posType[5]
#         return [time_ping, latitude, lat_dir, longitude, long_dir]
# #===================================================================