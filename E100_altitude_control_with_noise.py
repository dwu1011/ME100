"""
University of Michigan
ENG100-400
"""

import sys, platform
from pathlib import Path
if platform.system() == 'Darwin':
    airsim_install = '$HOME/AirSim'
else:
    airsim_install = 'C:\\AirSim'
sys.path.append(str(Path(airsim_install) / 'PythonClient'))
sys.path.append(str(Path(airsim_install) / 'PythonClient' / 'multirotor'))

############### import a few useful libraries ###########

#import setup_path 
import time
import numpy as np
import math
import matplotlib.pyplot as plt

############### establish the link to AirSim ###########

import airsim              # import AirSim API
import E100_functions      # import drone simulator library

dt = E100_functions.dt()  
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#### Copy and paste the above in your own flight controller #####

# Modify the following PID parameters to observe the altitude control performance
K_P = 1
K_I = 0.1
K_D = 1
#############

target_alt = 10
integration_term = 0
alt_flag = 1
throttle = 0.5  # initialize Throttle
alt_log = []
wind_flag = 1

start = time.time()

last_alt = 0
alpha = 1

while True:
           
    now = time.time()
    if now-start>15 and wind_flag==1:
        E100_functions.set_wind(client,0,0,-7)
        wind_flag=0
    elif now - start > 30 and now-start<33:
        target_alt = 0
    elif now - start >= 33:
        break
   
    #################### get sensor readings #####################      
    altitude = E100_functions.get_altitude(client) # read quadcopter's altitude 
    
    #################### Apply Low Pass #####################
    altitude = alpha*altitude + (1-alpha)*last_alt
    last_alt = altitude
    
    alt_log.append(altitude)
    plt.plot(alt_log)
    
    ##############################################################
    #Altitude hold (PID controller)##########################
    ##############################################################
    if alt_flag == 1:
        error_old = target_alt-altitude
        alt_flag = 0
    else:
        error_old = error    
    error= target_alt-altitude
    integration_term += error*dt
    differential_term = (error - error_old)/dt    
    if K_P*error + K_I*integration_term + K_D*differential_term <0:
        throttle = 0
    else:
        throttle = 0.5*(K_P*error + K_I*integration_term + K_D*differential_term)
    if throttle >= 1:
        throttle = 1
        
    ####### change the engine throttle ###################################
    E100_functions.set_quadcopter(client,0,0,0,throttle)
        
#### Copy and paste the following in your own flight controller #####    

############### release the link to AirSim ###########
client.armDisarm(False)
client.enableApiControl(False)