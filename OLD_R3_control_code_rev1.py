# -*- coding: utf-8 -*-
"""
Created on Wed Jan 29 16:16:34 2020

@author: Alexander Vanden Bussche
"""
#Import libraries
import numpy as np
import scipy as sp
import scipy.stats as st
import csv
import matplotlib.pyplot as plt
#==============================================================================
#Setup of the control variables

num_bat_s = 4 #Number in series
num_bat_p = 4 #number in parallel
bat_cell_nom = 3.6 #Volts
cell_energy_cap = 2.5 #Ah

bat_voltage = num_bat_s * bat_cell_nom 
bat_energy_full = num_bat_p*cell_energy_cap #Ah

#Constants of PID
Kp = 0.01
Ki = 0.01
Kd = 0.01
Ts = 0.01000 #time step = 0.01s
stop = 0
time = 0.0
pos = [0, 0, 0] #xyz
des_pos = [5, 5, 0] #x y z 

#Initialize errors
err_int_x = 0
err_int_y = 0
#err_x = 0
#err_y = 0
err_der_x = 0
err_der_y = 0
errorX = 0
errorY = 0
#Inputs from sensors

#Outputs to motor controllers

#error = (reference position – actual position) 
#integrated_error = integrated_error + error x Ts 
#error_derivative = (error – previous_error)/Ts
#control voltage = KBpB x error + Ki x integrated_error + Kd x error_derivative 


while stop==0:
    errorX_old = errorX
    errorY_old = errorY
    
    errorX = des_pos[0] - pos[0]
    errorY = des_pos[1] - pos[1]
    
    err_int_x = err_int_x + errorX*Ts 
    err_int_y = err_int_y + errorY*Ts
    
    err_der_x = (errorX - errorX_old)/Ts
    err_der_Y = (errorY - errorY_old)/Ts
    
    pos[0] = pos[0] + errorX/5
    pos[1] = pos[1] + errorY/5
    
    V_out_x = Kp * errorX + err_int_x*Ki + err_der_x*Kd
    V_out_y = Kp * errorY + err_int_y*Ki + err_der_y*Kd
    
    time = time+Ts
    print(V_out_x, "\t\t", V_out_y)
    if errorX <= 1e-3: 
        if errorY <= 1e-3:
            stop = 1
        
