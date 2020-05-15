import serial
import time
import math
#ser = serial.Serial('/dev/ttyACM0', 9600)

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
# Co-Authors: Kelly Low, Nathan Chow
# For questions or concerns, please call:
# (312) 809 1021
# or email:
# vand1090@umn.edu
# Spring 2020
#==========================================================

#in itialize sensors and buses
camera = PiCamera()
restart = True #unused at this point
i2c = busio.I2C(board.SCL, board.SDA) #initialize the i2c bus
bus = smbus.SMBus(1) #second i2c bus, should eventually be merged with the above. 
duino_address = 0x04 #defines i2c adress for the arduino

#with the following libraries, be aware of the restrictions of open source code
tof_sensor = adafruit_vl53l0x.VL53L0X(i2c) #library downloaded from Adafruit
compass = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)

#hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) # create MarvelmindHedge thread
#hedge.start() # start thread
LARGE_FONT= ("Verdana", 12)

#Direct Drive mode key press flags
w = 0
s = 0
a = 0
d = 0
shift = 0
dm2delay = 2

# Emergency Stop flag
StopDead = 0

# Drive mode constants for easier reading (Refer to Drive Functions.txt)
forward = [1, 7]
reverse = [2, 8]
forLeft = [3, 9]
forRight = [4, 10]
revLeft = [5, 11]
revRight = [6, 12]
stopLeft = [3, 9]
stopRight = [4, 10]
stopSlow = [14, 13]
Cspin = 15
# Dire is direction, 1 is forward, 0 is reverse, determines which slow stop
dire = 1

#semi-auto vars, do not mess with
currentPos = []
currentX = 0
currentY = 0
currentZ = 0
destinationReached = False
theta_transform = 0
#==========================================================
#Functions

def standby(num):
    #use as a pause between commands for debugging.
    #can also be used for drive mode 2
    time.sleep(num)
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
    camera.start_preview(fullscreen=False,window=(10,150,500,500)) #xywh

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

def dm3(goalX, goalX2, goalX3, goalY, goalY2, goalY3):
    driving(0)
    calibrate()
    print("Drive Mode 3")
    x_points = [goalX, goalX2, goalX3]
    y_points = [goalY, goalY2, goalY3]

    #get starting poisition
    findMe() #sets values of currentX and currentY

    #can use these to zero out
    x_origin = currentX
    y_origin = currentY

    num_waypoints = len(x_points)

    for i in range(0, num_waypoints):
        x_goal = x_points[i] - currentX
        y_goal = y_points[i] - currentY

        goal_heading = vector_2_degrees(x_goal, y_goal)
        current_heading = get_heading(compass)

        while(isHeading(current_heading, goal_heading) == False):
            #spin clockwise. Make better in future versions
            driving(15)
            current_heading = get_heading(compass)
            print(current_heading - goal_heading)
        standby(0.5) #pause to let software catch up
        driving(13)#stop the rover motion

        findMe() #check position again, recalculate goal
        if(x_goal == currentX and y_goal == currentY):
            posReached = True
        else:
            posReached = False

        while(posReached == False):
            driving(1)
            standby(0.1)
            findMe()
            if(x_goal == currentX and y_goal == currentY):
                posReached = True
            else:
                posReached = False
        print('Position ', i, ' Reached')





    finishCoords = transformMatrix(goalX, goalY)
    goalX_trans = finishCoords[0]
    goalY_trans = finishCoords[1]

    while(destinationReached == False):
        findMe()
        if(goalX == currentX and goalY == currentY):
            print('Success')
            drive = False
            driving(0)
            break


        deltaX = abs(currentX-goalX)
        deltaY = abs(currentY-goalY)

        hypotenuse = calcDist(deltaX, deltaY)
        driving(1)
    #Drive mode 3 is automatic mode
    #auto mode
    #read in instructions
    #write function to calculate heading
    #write function to check environment
#==========================================================
#Autonomous mode functions
def transformMatrix(xCoord, yCoord):
    #starting xy coord
    start = [xCoord, yCoord]
    trans_matrix = [math.sin(theta_transform), math.cos(theta_transform), 0],[-1*math.cos(theta_transform), math.sin(theta_transform), 0], [0,0,1]
    finishCoords = trans_matrix * start
    return finishCoords

def calibrate():
    #use this function to correlate N,S,E,W with +/- X and Y
    print("Calibrating")
    # Need to make transformation matrix
    # Drive in a square - north, then east, then south then west

    # turn to point North
    currentDir = get_heading(compass)
    isNorth(currentDir)
    while(isNorth(currentDir) == False):
        #spin clockwise. Update this in future verions
        driving(15)
        standby(0.5)
        currentDir = get_heading(compass)
        print(currentDir)
    driving(13)
    #now the rover is pointing North
    #get starting position for calibrations
    findMe()
    x_cal_1 = currentX
    y_cal_1 = currentY

    #drive forward
    driving(1)
    standby(1.5)
    driving(0)

    #get end position
    findMe()
    x_cal_2 = currentX
    y_cal_2 = currentY

    #find theta in XY space
    delX = x_cal_2-x_cal_1
    delY = y_cal_2-y_cal_1
    hyp = 1#calcDist(delX, delY)
    thetaXYspace = math.degrees(math.asin(delY/hyp))

    #this will be used to transform gu=oal location to real spaces
    theta_transform = thetaXYspace-get_heading(compass)

    #go back to starting locations
    driving(2)
    standby(1.5)
    driving(0)


def isNorth(currentDir):
    #put in a +- buffer of 2 degrees
    if(currentDir > 98  and currentDir < 102):
        return True
    else:
        return False

def isHeading(currentDir, goalDir):
    #put in a +- buffer of 2 degrees
    if(currentDir > goalDir-2  and currentDir < goalDir+2):
        return True
    else:
        return False

def findMe():
    #this function finds the rover in the IPS system
    #hedge.print_position()
    currentPos = hedge.position()
    currentX = currentPos[0]
    currentY = currentPos[1]
    currentZ = currentPos[2]

def checkObs():
    rangeMin = 300 #Rover allowed no closer than this (mm)
    rangeAvoid = 500
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
        driving(13)
        print("No obstacles")
        return 0

def avoidObs(severity):
    #this function tries to move the rover arund an obstacle
    print("avoiding obstacles")

    if(severity == 2): #meaning it's about to hit the obstable
        driving(0) #stop the rover, then back it up
        #back & right
        driving(2) #change to correct value - look up
        standby(1)
        turnDegrees(3)
        checkObs()

    elif(severity == 1): #has room to change course w/o stopping
        #slow down, start turning
        turnDegrees(3) # forward left
        standby(1)
        checkObs()

    else:
        print("no obstacles to avoid")

# calc total distance to target
def calcDist(deltaX, deltaY):
    return math.sqrt(math.pow(deltaX, 2) + math.pow(deltaY,2))

def turnDegrees(numDeg):
    currentDir = get_heading(compass)
    goalHeading = currentDir + numDeg
    while(currentDir <= goalHeading):
        driving(4)
        standby(1)
        currentDir = get_heading(compass)

# compass code
def vector_2_degrees(x, y):
    angle = math.degrees(math.atan2(y, x))
    if angle < 0:
        angle += 360
    return angle


def get_heading(_sensor):
    magnet_x, magnet_y, _ = _sensor.magnetic
    #mag_x, mag_y, mag_z = sensor.magnetic
    print('Magnetometer (gauss): ({0:10.3f}, {1:10.3f})'.format(magnet_x, magnet_y))
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
#children of GUI
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
        
        # Debouncer handoff for keypresses, prevents crash due to key holding
        self.w = Debouncer(self.on_w,self.off_w)
        self.a = Debouncer(self.on_a,self.off_a)
        self.s = Debouncer(self.on_s,self.off_s)
        self.d = Debouncer(self.on_d,self.off_d)
        self.x = Debouncer(self.on_x,self.off_x)
        self.j = Debouncer(self.on_j,self.off_j)
        
        # Keypress keybinds
        self.label.bind("<Shift_L>", self.shift.released)
        self.label.bind("<w>", self.w.pressed)
        self.label.bind("<a>", self.a.pressed)
        self.label.bind("<s>", self.s.pressed)
        self.label.bind("<d>", self.d.pressed)
        self.label.bind("<x>", self.x.pressed)
        self.label.bind("<j>", self.j.pressed)

        # Keypress release binds
        self.label.bind("<KeyRelease-Shift_L>", self.shift.released)
        self.label.bind("<KeyRelease-w>", self.w.released)
        self.label.bind("<KeyRelease-a>", self.a.released)
        self.label.bind("<KeyRelease-s>", self.s.released)
        self.label.bind("<KeyRelease-d>", self.d.released)
        self.label.bind("<KeyRelease-x>", self.x.released)
        self.label.bind("<KeyRelease-j>", self.j.released)
        # give keyboard focus to the label by default, and whenever
        # the user clicks on it
        self.label.focus_set()
        self.label.bind("<1>", lambda event: self.label.focus_set())
        self.label.grid(column = 1, row = 5)
    
    # Hi-speed flag, turns on "shift" flag on press an off on release
    def on_shift(self,event):
        global shift
        shift = 1
    
    def off_shift(self, event):
        global shift
        shift = 0
    
    # W-key, drives forward if pressed alone and forward+left/right if a/d are pressed
    def on_w(self, event):
        self.label.configure(text="Forward")
        if dm2:
            time.sleep(dm2delay)
        global w, a, s, d, shift, dire
        dire = 1
        w = 1
        if a == 1:
            driving(forLeft[shift])
        elif d == 1:
            driving(forRight[shift])
        else:
            driving(forward[shift])
            
    # Checks to return to other drive directions when released        
    def off_w(self, event):
        if dm2:
            time.sleep(dm2delay)
        global w, a, s, d, shift
        w = 0
        if a == 1:
            driving(stopLeft[shift])
        elif d == 1:
            driving(stopRight[shift])
        elif s == 0:
            driving(stopSlow[1])

    # A key, turns rover left, since there is no c-wise and cc-wise spin, drives forward left by default
    def on_a(self, event):
        self.label.configure(text="Left")
        if dm2:
            time.sleep(dm2delay)
        global w, a, s, d, shift
        a = 1
        if w == 1:
            driving(forLeft[shift])
        elif s == 1:
            driving(revLeft[shift])
        else:
            driving(stopLeft[shift])
   
    # Returns to other drive functions when released
    def off_a(self, event):
        if dm2:
            time.sleep(dm2delay)
        global w, a, s, d, shift
        a = 0
        if w == 1 and s == 1:
            if dire == 0:
                driving(forward[shift])
            elif dire == 1:
                driving(reverse[shift])
        elif w == 1:
            driving(forward[shift])
        elif s == 1:
            driving(reverse[shift])
        else:
            # Since there is no turn without moving, slow stop is used
            # driving(stopDead)
            driving(stopSlow[dire])
        
            
    # S key, reverse when pressed checks a/d to allow for reverse left/right 
    def on_s(self, event):
        self.label.configure(text="Backward")
        if dm2:
            time.sleep(dm2delay)
        global w, a, s, d, shift, dire
        dire = 0
        s = 1
        if a == 1:
            driving(revLeft[shift])
        elif d == 1:
            driving(revRight[shift])
        else:
            driving(reverse[shift])
    
    # Checks if other keys are pressed and returns to turn, will return to reverse left/right when a/d is still held after
    # s is released to prevent jerking
    def off_s(self, event):
        if dm2:
            time.sleep(dm2delay)
        global w, a, s, d, shift
        s = 0
        if a == 1:
        # If standstill turns are implemented, replace revLeft with stopLeft,
        # This is to prevent jerking since there is no stop left or stop right
            driving(revLeft[shift])
        elif d == 1:
        # If standstill turns are implemented, replace revRight with stopRight,
        # This is to prevent jerking since there is no stop left or stop right
            driving(revRight[shift])
        elif w == 0:
            driving(stopSlow[0])

    # D key, turns rover rightnce there is no c-wise and cc-wise spin, drives forward right by default
    def on_d(self, event):
        self.label.configure(text="Right")
        if dm2:
            time.sleep(dm2delay)
        global w, a, s, d, shift
        d = 1
        if w == 1:
            driving(forRight[shift])
        elif s == 1:
            driving(revRight[shift])
        else:
            driving(stopRight[shift])
            
    # Returns to forward or reverse when released
    def off_d(self, event):
        if dm2:
            time.sleep(dm2delay)
        global w, a, s, d, shift
        d = 0
        if w == 1 and s == 1:
            if dire == 0:
                driving(forward[shift])
            elif dire == 1:
                driving(reverse[shift])
        elif w == 1:
            driving(forward[shift])
        elif s == 1:
            driving(reverse[shift])
        else:
            # Since there is no turn without moving, slow stop is used
            # driving(stopDead)
            driving(stopSlow[dire])
            
    # Emergency stop, resets all key flags to off
    def on_x(self, event):
        driving(stopDead)
        global w, a, s, d, shift, dire 
        shift = 0
        w = 0
        a = 0
        s = 0
        d = 0
        dire = 0
    
    # Ensures keys flags are off
    def off_x(self, event):
        global w, a, s, d, shift, dire 
        shift = 0
        w = 0
        a = 0
        s = 0
        d = 0
        dire = 0

    # Clockwise spin autonomous debug test
    def on_j(self, event):
        self.label.configure(text = "Spin")
        driving(15)
    def off_j(self, event):
        driving(13)
        
    # Displays current/last pressed key on GUI
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

#Semi auto drive mode
class dm3Page(tk.Frame):
    #initialize semi auto window
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
        self.inputXField.grid(column = 2, row = 4)
        self.inputXField2 = tk.Entry(self)
        self.inputXField2.grid(column = 3, row = 4)
        self.inputXField3 = tk.Entry(self)
        self.inputXField3.grid(column = 4, row = 4)
    def inputXLabel(self):
        self.inXlabel = tk.Label(self, text="Input Waypoint X Coords", font=LARGE_FONT)
        self.inXlabel.grid(column=1, row = 4)
    #get y goal
    def inputYGoal(self, controller):
        self.inputYField = tk.Entry(self)
        self.inputYField.grid(column = 2, row = 5)
        self.inputYField2 = tk.Entry(self)
        self.inputYField2.grid(column = 3, row = 5)
        self.inputYField3 = tk.Entry(self)
        self.inputYField3.grid(column = 4, row = 5)
    def inputYLabel(self):
        self.inYlabel = tk.Label(self, text="Input Waypoint Y Coordinates", font=LARGE_FONT)
        self.inYlabel.grid(column=1, row = 5)

    #general software stuff
    def swKill(self):
        endCam()
        kill()
    def startBtn(self): #makes stat button
        self.start = tk.Button(self, text = 'START', command = self.startCommand)
        self.start.grid(column = 1, row = 3)
    # this is when auto mode starts
    def startCommand(self):
        goalX = float(self.inputXField.get())
        goalX2 = float(self.inputXField2.get())
        goalX3 = float(self.inputXField3.get())
        goalY = float(self.inputYField.get())
        goalY2 = float(self.inputYField2.get())
        goalY3 = float(self.inputYField3.get())
        print(goalX, goalX2, goalX3)
        print(goalY, goalY2, goalY3)
        startCam()
        dm3(goalX, goalX2, goalX3, goalY, goalY2, goalY3)

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
