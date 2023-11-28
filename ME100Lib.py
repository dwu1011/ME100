import math
from constants import distance_threshold
from enum import Enum

decision_thresh = 100 #choose a number that makes sense

class PID:
    def __init__ (self, KP: float, KI: float, KD: float, dt:float):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.dt = dt
        self.lastErr = 0
        self.totalErr = 0

    def calculate(self, current: float, setpoint: float) -> float:
        err = setpoint - current

        self.totalErr += err * self.dt

        derivedErr = (err - lastErr)/self.dt

        output = self.KP * err + self.KI * self.totalErr + self.KD * derivedErr

        lastErr = err

        return clamp(output, 0, 1)
    
    def calculate(self, measurement: float) -> float:
        return self.calculate(measurement, 0)
    

def find_lpf(alpha: float, dat: float, prev: float) -> float:
    return alpha*dat + (1-alpha)*prev

def clamp(value, min: float, max: float) -> float:
    return math.max(min, math.min(value, max))

Direction = Enum('Direction', ['LEFT', 'RIGHT', 'STRAIGHT'])

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
def decide_turn(left_dist, right_dist) -> float:
    if left_dist >= decision_thresh and left_dist > right_dist:
        return Direction.LEFT
        # pass
    elif right_dist >= decision_thresh and right_dist >= left_dist:
        #turn right 
        return Direction.RIGHT
        # pass
    else:
        #keep going straight 
        return Direction.STRAIGHT
        # pass

def decide_altitude(up_dist, down_dist) -> float:
    if up_dist >= decision_thresh and up_dist > down_dist:
        #go up
        pass
    elif down_dist >= decision_thresh and down_dist >= up_dist:
        #go down
        pass
    else:
        #maintain altitude 
        pass
    
    

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
        return current_yaw + 90 # is this supposed to return the yaw or set the yaw 
    elif(decision is Direction.RIGHT):
        return current_yaw - 90
    return current_yaw