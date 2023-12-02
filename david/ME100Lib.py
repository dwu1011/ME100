import math
from constants import distance_threshold
from enum import Enum

decision_thresh = 15 #choose a number that makes sense

class PID:
    def __init__ (self, KP: float, KI: float, KD: float, dt:float):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.dt = dt
        self.lastErr = 0
        self.totalErr = 0

    def calculateWithSetpoint(self, current: float, setpoint: float) -> float:
        err = setpoint - current

        self.totalErr += err * self.dt

        derivedErr = (err - self.lastErr)/self.dt
        
        # print(derivedErr, err)
        output = self.KP * err + self.KI * self.totalErr + self.KD * derivedErr
        #print("Kp is " + str(self.KP * err))

        self.lastErr = err

        return output
    
    def calculate(self, measurement: float) -> float:
        return self.calculateWithSetpoint(measurement, 0)
    

def find_lpf(alpha: float, dat: float, prev: float) -> float:
    return alpha*dat + (1-alpha)*prev

def clamp(value, mi: float, ma: float) -> float:
    return max(mi, min(value, ma))

Direction = Enum('Direction', ['LEFT', 'RIGHT', 'STRAIGHT', 'UP', 'DOWN', 'FRONT', 'BACK'])

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
    else:
        #keep going straight 
        return Direction.STRAIGHT
        # pass

def decide_altitude(up_dist, down_dist) -> Direction:
    if up_dist >= decision_thresh and up_dist > down_dist:
        #go up
        return Direction.UP
    elif down_dist >= decision_thresh and down_dist >= up_dist:
        #go down
        return Direction.DOWN
    else:
        #maintain altitude 
        return Direction.STRAIGHT
    
def decide_all(up, down, left, right) -> Direction:
    d = decide_turn(left, right)
    if(d != Direction.STRAIGHT): return d
    d = decide_altitude(up, down)
    return d

def check_directions(front, left, right, rear):
    if(front >= decision_thresh): return Direction.FRONT
    if(left >= decision_thresh): return Direction.LEFT
    if(right >= decision_thresh): return Direction.RIGHT
    if(rear >= decision_thresh): return Direction.BACK
    
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
        return (current_yaw - 90)%360 # is this supposed to return the yaw or set the yaw 
    elif(decision is Direction.RIGHT):
        return (current_yaw + 90)%360
    elif(decision is Direction.BACK):
        return (current_yaw + 180)%360
    return current_yaw
