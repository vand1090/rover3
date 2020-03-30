#!/usr/bin/pyth
# -*- coding:utf-8 -*-
import smbus
import time
#import busio

# address list
duino_address = 0x04

bus = smbus.SMBus(1)

def writeNum(val):
    bus.write_byte(duino_address, val)
    return -1

def readNum():
    num = bus.read_byte(duino_address)
    return num

#while True:
var = int(input("Enter 1-9: "))
    #if not var

writeNum(var)
print("RPI sent ", var)
time.sleep(0.5)
num = readNum()
print("RPI Received", num)
time.sleep(0.5)