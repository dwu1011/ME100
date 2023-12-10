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
from enum import Enum

############### establish the link to AirSim ###########

import airsim              # import AirSim API
import E100_functions      # import drone simulator library


############## ME100 Libraries ##############
from ME100Lib import PID, find_lpf, Direction, decide_turn, decide_vertical, determine_yaw, clamp, get_custom_lidar
from constants import Altitude_Constants as AConst, alpha_lidar, Pitch_Constants as PConst
from constants import Roll_Constants as RConst

dt = E100_functions.dt()  
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

################ ALTITUDE INITIALIZATIONS ################
alpha_alt = AConst.alpha_alt
target_alt = AConst.target_alt
throttle = 0.5  # initialize Throttle
alt_pid = PID(AConst.altitude_pid_gains[0],
                AConst.altitude_pid_gains[1],
                AConst.altitude_pid_gains[2],
                dt)

################ PITCH INITIALIZATIONS ################
target_front_dist = PConst.target_front_dist
pitch_pid = PID(PConst.pitch_pid_gains[0],
                    PConst.pitch_pid_gains[1],
                    PConst.pitch_pid_gains[2],
                    dt)
desired_pitch = 0

################ ROLL INITIALIZATIONS ################
target_right_dist = 5
roll_pid = PID(RConst.roll_pid_gains[0],
                    RConst.roll_pid_gains[1],
                    RConst.roll_pid_gains[2],
                    dt)
desired_roll = 0 
desired_yaw = 270

sustain = 0

sustain_vert = 0

start = time.time()

altitude_sensor_flag = 1
Lidar_sensor_flag = 1

x_pos = []
y_pos = []
lat_error = []

State = Enum('State', ['M', 'T', 'B']) # STATE DESCRIBES THE CURRENT HANDLED INSTRUCTION

drone_state = State.M

isStraight = True
wasTop = False
isLeveled = True
lastTime = time.time()

while True:
           
    now = time.time()
    if now - start > 298:
        target_alt = 0
    if now - start > 300:   
        break
    
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
    altitude = find_lpf(alpha_alt, altitude_n0, altitude_n1) # apply the low-pass filter
    altitude_n1 = altitude
    
    roll, pitch, yaw = E100_functions.get_orientation(client) # read quadcopter's attitude
    
    front, right, left, back = E100_functions.get_lidars(client)    # read LIDAR readings
    
    top = get_custom_lidar(client) # retrieve custom LiDAR (top sensor)

    # apply low-pass filters for each sensor
    front = find_lpf(alpha_lidar, front, front1)
    right = find_lpf(alpha_lidar, right, right1)
    left = find_lpf(alpha_lidar, left, left1)
    top = find_lpf(alpha_lidar, top, top1)
    
    
    front1 = front
    right1 = right
    left1 = left
    top1 = top

    
    if (now - start) >= 2: # Only start the moving program after 2 seconds
        neg_const = 0
        # Linear Velocity determined if the drone is going in the X or Y direction 
        linear_vel = abs(
            E100_functions.get_linear_velocity(client)
                [0 if math.isclose(abs(desired_yaw % 180), 90, abs_tol=1) else 1]
            )
        
        p_gain = 0.6 * linear_vel # P Controller for velocity
        if isStraight:
            neg_const = (p_gain * linear_vel) # it will only take effect if the drone is going down a straight
        desired_pitch = pitch_pid.calculate(front, target_front_dist) + neg_const # The desired_pitch is determined through the addition of the 2
    
    
    # Determine the decision to turn laterally
    d = decide_turn(left, right)
    # Determine the decision to turn vertically
    dv = decide_vertical(top, altitude)
    
    # First determine whether the drone is to go up or down
    if dv == Direction.TOP and isLeveled:
        # This concept of sustain is used a lot, it essentially forces the decision to be declarative rather than sporadic
        sustain_vert += 1
        if sustain_vert >= 20: # Here it requires 20 iterations to declare it has to go up
            isLeveled = False
            drone_state = State.T
    else:
        sustain_vert = 0
        
    if dv == Direction.BOTTOM:
        wasTop = False
        isLeveled = False
        
    if dv is None:
        isLeveled = True
        
    # State.M references normal maze travel, and is only active when the drone_state is not vertical
    if drone_state == State.M:

        # Keep hugging the ceiling until the floor is "close" enough
        if wasTop and dv != Direction.BOTTOM: 
            throttle = alt_pid.calculate(9, top)
            throttle = clamp(throttle, 0, 1)
            if math.isclose(top, altitude, abs_tol=8):
                wasTop = False
                
        else:
            throttle = alt_pid.calculate(altitude, target_alt)
            throttle = clamp(throttle, 0, 1)
        
        # When turning, hug the opposite wall of the turn (rather than center)
        if not isStraight and not math.isclose(left, right, abs_tol=4):
            if left < right:
                desired_roll = roll_pid.calculate(left, 5)
            else:
                desired_roll = -roll_pid.calculate(right, 5)
            desired_roll = math.degrees(0.11*np.tanh(desired_roll))

        else: # Otherwise use the basic centering algorithm
            desired_roll = roll_pid.calculate(left - right)
            desired_roll = math.degrees(0.075*np.tanh(desired_roll))
        
        # Logic to determine whether the drone has straightened our
        # Note the similar usage of "sustain" as described earlier
        if d == Direction.STRAIGHT:
            sustain += 1
            if sustain >= 20:
                isStraight = True
        
        # As soon as a lateral turn is available, initiate the instruction to turn
        elif d != Direction.STRAIGHT and isStraight:
            isStraight = False
            sustain = 0
            desired_yaw = determine_yaw(d, desired_yaw)
    
    else: # The drone needs to go up and look for an opportunity to switch to State.M
          # In order to switch to State.M it needs to find an available turn
        wasTop = True
        throttle = alt_pid.calculate(target_alt, top)
        throttle = clamp(throttle, 0, 1)
        desired_roll = roll_pid.calculate(left - right)
        desired_roll = math.degrees(0.075*np.tanh(desired_roll))
        if math.isclose(top, 6.5, abs_tol=2):
            deci = decide_turn(left, right)
            desired_yaw = determine_yaw(deci, desired_yaw)
            drone_state = State.M
            isStraight = False
            
    desired_yaw %= 360   

############### release the link to AirSim ###########
client.armDisarm(False)
client.enableApiControl(False)