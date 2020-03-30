import serial
import time
#ser = serial.Serial('/dev/ttyACM0', 9600)
import tkinter as tk
import cv2
from picamera import PiCamera
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
#==========================================================
#Functions

def standby():
    #use as a pause between commands for debugging.
    #can also be used for drive mode 2
    time.sleep(5)
    #return True
def getRange():
   # print('Range: {}mm'.format(tof_sensor.range))