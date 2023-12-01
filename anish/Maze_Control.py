'''
THIS ONE IS ANISH"S VERISON, DAVID SHOULD ALSO UPLOAD HIS
'''

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


############## ME100 Libraries
from ME100Lib import PID, find_lpf, Direction, decide_turn, determine_yaw, clamp, get_custom_lidar
from constants import Altitude_Constants as AConst, alpha_lidar, Pitch_Constants as PConst
from constants import Roll_Constants as RConst

dt = E100_functions.dt()  
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#### Copy and paste the above in your own flight controller #####

alpha_alt = AConst.alpha_alt
# alpha_lidar = a_l
###### TODO ALTITUDE IS ALREADY DONE, JUST DO THE SAME FOR THE OTHER 2 SETS OF CONSTANTS, MAKES IT EASIER TO TUNE, you need to edit "constants.py" as well
target_alt = AConst.target_alt
# K_P = 1
# K_I = 0.
# K_D = 2
throttle = 0.5  # initialize Throttle
alt_pid = PID(AConst.altitude_pid_gains[0],
                AConst.altitude_pid_gains[1],
                AConst.altitude_pid_gains[2],
                dt)
##############################################
target_front_dist = PConst.target_front_dist
pitch_pid = PID(PConst.pitch_pid_gains[0],
                    PConst.pitch_pid_gains[1],
                    PConst.pitch_pid_gains[2],
                    dt)
desired_pitch = 0
##############################################
target_right_dist = 5
roll_pid = PID(RConst.roll_pid_gains[0],
                    RConst.roll_pid_gains[1],
                    RConst.roll_pid_gains[2],
                    dt)
desired_roll = 0 
desired_yaw = 270

sustain = 0

start = time.time()

altitude_sensor_flag = 1
Lidar_sensor_flag = 1

x_pos = []
y_pos = []
lat_error = []

isStraight = True

while True:
           
    now = time.time()
    if now - start > 88:
        target_alt = 0
    if now - start > 90:   
        break
    
    #control signal 
    #print("throttle:", throttle, "\ndesired pitch:", desired_pitch, "\ndesired roll:", desired_roll, "\nDirection Decision:", str(d), "\nDesired_yaw:", desired_yaw, "\n------------------------")
    E100_functions.set_quadcopter(client,desired_roll,desired_pitch,desired_yaw,throttle)
    
    x,y = E100_functions.get_XY(client)
    x_pos.append(x)
    y_pos.append(y)
   
    #################### get sensor readings #####################
    if altitude_sensor_flag==1:
        altitude_n1=0
        altitude_sensor_flag=0
    if Lidar_sensor_flag == 1:
        Lidar_sensor_flag = 0
        front1, right1, left1, back1 = E100_functions.get_lidars(client)
        top1 = get_custom_lidar(client)
               
    altitude_n0 = E100_functions.get_altitude(client) # read quadcopter's altitude 
    altitude = find_lpf(alpha_alt, altitude_n0, altitude_n1)
    altitude_n1 = altitude
    
    roll, pitch, yaw = E100_functions.get_orientation(client) # read quadcopter's attitude
    
    front, right, left, back = E100_functions.get_lidars(client)    # read LIDAR readings
    
    top = get_custom_lidar(client)

    front = find_lpf(alpha_lidar, front, front1)
    right = find_lpf(alpha_lidar, right, right1)
    left = find_lpf(alpha_lidar, left, left1)
    
    top = find_lpf(alpha_lidar, top, top1)
    
    
    front1 = front
    right1 = right
    left1 = left
    top1 = top
    
    throttle = alt_pid.calculate(altitude, target_alt)
    throttle = clamp(throttle, 0, 1)
    if (now - start) >= 2:
        neg_const = 0
        #p_gain = 0.25
        linear_vel = abs(E100_functions.get_linear_velocity(client)[0 if math.isclose(abs(desired_yaw % 360), 90, abs_tol=1) else 1])
        p_gain = 0.6 * linear_vel
        #if linear_vel >= 3:
         #   p_gain = 2
        if isStraight :
            neg_const = (p_gain * linear_vel)
        desired_pitch = pitch_pid.calculate(front, target_front_dist) + neg_const
        #lat_error.append(p_gain * abs(E100_functions.get_linear_velocity(client)[0 if math.isclose(abs(desired_yaw % 360), 90, abs_tol=1) else 1]))
    
    #lat_error.append(altitude)
    
    
    d = decide_turn(left, right)
    
    if not isStraight and not math.isclose(left, right, abs_tol=4):
        if left < right:
            desired_roll = 1.2 * roll_pid.calculate(left, 3)
        else:
            desired_roll = 1.2 * -roll_pid.calculate(right, 3)
        desired_roll = math.degrees(0.075*np.tanh(desired_roll))
    else:
        desired_roll = roll_pid.calculate(left - right)
        desired_roll = math.degrees(0.075*np.tanh(desired_roll))
        
    lat_error.append(top)
        
    
    if d == Direction.STRAIGHT:
        sustain += 1
        if sustain >= 20:
            isStraight = True
    elif d != Direction.STRAIGHT and isStraight:
        isStraight = False
        sustain = 0
        desired_yaw = determine_yaw(d, desired_yaw)
        
    #print("DD=", str(d), "\nIS", isStraight, "\ndy=", desired_yaw)

    plt.plot(lat_error)


#plt.scatter(x_pos,y_pos)

#### Copy and paste the following in your own flight controller #####    

############### release the link to AirSim ###########
client.armDisarm(False)
client.enableApiControl(False)