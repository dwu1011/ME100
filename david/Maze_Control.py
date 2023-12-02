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


############## ME100 Libraries
from ME100Lib import PID, find_lpf, Direction, decide_turn, determine_yaw, decide_all, check_directions
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
bottom = E100_functions.get_SONAR(client)
top = E100_functions.get_top_lidar(client)

target_alt = (bottom+top)/2
# target_alt = 10
# K_P = 1
# K_I = 0.
# K_D = 2
throttle = 0.5  # initialize Throttle
alt_pid = PID(AConst.altitude_pid_gains[0],
                AConst.altitude_pid_gains[1],
                AConst.altitude_pid_gains[2],
                dt)
##############################################
target_front_dist = 7
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
desired_yaw = 0
target_yaw = 0

P_yaw = 0.75

start = time.time()

altitude_sensor_flag = 1
Lidar_sensor_flag = 1

x_pos = []
y_pos = []
lat_error = []
yaw_log = []
dyaw_log = []

alt_log = []
d = Direction.STRAIGHT

calc_alt = True

right_pid = False
left_pid = False
front_pid = True

twod = True

last = time.time()

while True:
           
    now = time.time()
    # if now - start > 20:
    #     #target_alt = 0
    #     ...
    
    if now - start > 240:   
        break
    if(calc_alt):
        bottom = E100_functions.get_SONAR(client)
        top = E100_functions.get_top_lidar(client)
        
        target_alt = (bottom+top)/2
        calc_alt = False
    #control signal 
    #print(throttle, desired_roll, desired_pitch, desired_yaw)
    E100_functions.set_quadcopter(client,desired_roll,desired_pitch,desired_yaw,throttle)
    
    # print(desired_roll, desired_pitch, desired_yaw,  throttle)
    
    x,y = E100_functions.get_XY(client)
    x_pos.append(x)
    y_pos.append(y)
    
   
    #################### get sensor readings #####################
    if altitude_sensor_flag==1:
        altitude_n1=0
        altitude_sensor_flag=0
    if Lidar_sensor_flag == 1:
        front1, right1, left1, back1 = E100_functions.get_lidars(client)
        
    
    altitude_n0 = E100_functions.get_altitude(client) # read quadcopter's altitude 
    altitude = find_lpf(alpha_alt, altitude_n0, altitude_n1)
    altitude_n1 = altitude
    
    alt_log.append(altitude_n0)
    # plt.plot(alt_log)
    
    roll, pitch, yaw = E100_functions.get_orientation(client) # read quadcopter's attitude
    
   
    # print(yaw, desired_yaw, target_yaw)
    
    print(roll, pitch, yaw, target_alt, altitude_n0, top)
    
    yaw_log.append(yaw)
    dyaw_log.append(desired_yaw)
    
    front, right, left, back = E100_functions.get_lidars(client)    # read LIDAR readings
    top = E100_functions.get_top_lidar(client)
    down = E100_functions.get_SONAR(client)
    
    # print(front, right, left, back)
    front = np.cos(math.radians(abs(desired_pitch)))*front
    
    front = find_lpf(alpha_lidar, front, front1)
    right = find_lpf(alpha_lidar, right, right1)
    left = find_lpf(alpha_lidar, left, left1)
    
    front1 = front
    right1 = right
    left1 = left
    
    throttle = max(0,min(1,alt_pid.calculateWithSetpoint(altitude, target_alt)))

    # desired_roll = roll_pid.calculateWithSetpoint(left, right)
    # desired_roll = -math.degrees(0.075*np.tanh(desired_roll))
    
    if(left_pid):
        desired_roll = roll_pid.calculateWithSetpoint(left, 5)
    elif(right_pid):
        desired_roll = roll_pid.calculateWithSetpoint(5, right)
        
    if(front_pid):
        desired_pitch = pitch_pid.calculateWithSetpoint(front, target_front_dist)
    
    
    desired_roll = math.degrees(0.075*np.tanh(desired_roll))
    desired_pitch = math.degrees(0.08*np.tanh(desired_pitch))
    
    
    if(math.isclose(left, 5, rel_tol=1e-2) and left_pid and now-last > 2):
        print("left stopping")
        desired_roll = 0;
        # right_pid = False
        left_pid = False
    
    if(math.isclose(right, 5, rel_tol=1e-2) and right_pid and now-last > 2):
        print("right stopping")
        desired_roll = 0;
        right_pid = False
        # left_pid = False
    
    
    x, y, z = E100_functions.get_linear_velocity(client)
    print(x,y,z)
    
    if(math.isclose(front, target_front_dist,rel_tol= 2e-2) and now-last > 3):
        print("CLOSE ENOUGH")
        d = decide_all(top, down, left, right)
        if(d == Direction.LEFT):
            last = now
            left_pid = False
            right_pid = True
        elif(d == Direction.RIGHT):
            last = now
            right_pid = False
            left_pid = True
        elif(d == Direction.UP and twod):
            print("GOING UPPP")
            twod = False
            last = now
            target_alt = altitude_n0 + top - 5
            # front_pid = False
            desired_pitch = 0
    else:
        d = Direction.STRAIGHT
    
    if(yaw < 0): yaw += 360
    if d == Direction.RIGHT or d==Direction.LEFT:
        print("starting to ROTATE")
        print(left_pid ,right_pid)
        
        desired_yaw = determine_yaw(d, yaw)
    
    if(not twod and  math.isclose(target_alt, altitude_n0, rel_tol = 1e-2)):
        check_direction = check_directions(front, left, right, back)
        desired_yaw = determine_yaw(check_direction, yaw)
        twod=True
    
    lat_error.append(right - left)

    # print(front)
    # plt.plot(lat_error)

plt.plot(yaw_log)
plt.plot(dyaw_log)
# plt.scatter(x_pos,y_pos)

#### Copy and paste the following in your own flight controller #####    

############### release the link to AirSim ###########
client.armDisarm(False)
client.enableApiControl(False)
