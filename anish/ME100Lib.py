from enum import Enum

import E100_functions

import numpy as np

decision_thresh = 12 #choose a number that makes sense

class PID:
    def __init__ (self, KP: float, KI: float, KD: float, dt:float):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.dt = dt
        self.lastErr = 0
        self.totalErr = 0

    def calculate(self, current: float, setpoint: float = None, dampening_constant: float = 0) -> float:
        err = 0
        if not setpoint is None:
            err = setpoint - current
        else:
            err = 0 - current

        self.totalErr += err * self.dt

        derivedErr = (err - self.lastErr)/self.dt
        

        output = self.KP * err + self.KI * self.totalErr + self.KD * (derivedErr)

        self.lastErr = err

        return output
    
    def set_p(self, p:float):
        self.KP = p
    

def find_lpf(alpha: float, dat: float, prev: float) -> float:
    return alpha*dat + (1-alpha)*prev

def clamp(value, mi: float, ma: float) -> float:
    return max(mi, min(value, ma))

Direction = Enum('Direction', ['LEFT', 'RIGHT', 'STRAIGHT', 'BACK', 'TOP', 'BOTTOM'])

'''
Purpose: This function's goal is to determine whether it's a good idea to turn left or right
Parameters:
    left_dist (float): the LiDAR perceived distance to the left wall
    right_dist (float): the LiDAR perceived distance to the right wall
Logic:
    If the distance is above some constant threshold (defined at imports), then that's the correct direction 
    up_dist and down_dist need to be defined, I just chose names

Actions:
I described the actions below, I am not sure what the code looks like.
'''
def decide_turn(left_dist, right_dist) -> Direction:
    if left_dist >= decision_thresh and left_dist > right_dist:
        return Direction.LEFT
        # pass
    elif right_dist >= decision_thresh and right_dist >= left_dist:
        #turn right 
        return Direction.RIGHT
        # pass
    #elif back_dist >= decision_thresh:
    #   return Direction.BACK
    else:
        #keep going straight 
        return Direction.STRAIGHT
    
    
def decide_vertical(top_dist, bot_dist) -> Direction:
    if top_dist >= decision_thresh and top_dist >= bot_dist:
        return Direction.TOP
    elif bot_dist >= decision_thresh and bot_dist >= top_dist:
        return Direction.BOTTOM
    return None

'''
Purpose: This function's goal is to determine the yaw that needs to be used based on the direction
Keep in mind that yaw increases in the counter clockwise direction

Parameters:
    decision (Direction): This is the direction chosen, this is in the form of Direction.LEFT, or Direction.RIGHT
    current_yaw (float): This is the current yaw being tracked on the drone

I didn't do anything here because I was not sure how this was supposed to work.
'''
def determine_yaw(decision, current_yaw) -> float:
    if(decision is Direction.LEFT):
        return current_yaw - 90 # is this supposed to return the yaw or set the yaw 
    elif(decision is Direction.RIGHT):
        return current_yaw + 90
    return current_yaw


def get_custom_lidar(client,noise_mag = 1):
    lidardata_new = client.getLidarData(lidar_name="LidarSensorNew")
    points5= E100_functions.get_lidarData_points(lidardata_new)
    with E100_functions.warnings.catch_warnings():
        E100_functions.warnings.simplefilter("ignore", category=RuntimeWarning)
        new_dist= np.linalg.norm(np.mean(points5, axis=0))
    new_dist += noise_mag*E100_functions.noise_gen(new_dist)
    return new_dist
