import math
from constants import distance_threshold
from enum import Enum

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

Direction = Enum('Direction', ['LEFT', 'RIGHT'])

'''
Purpose: This function's goal is to determine whether it's a good idea to turn left or right
Parameters:
    left_dist (float): the LiDAR perceived distance to the left wall
    right_dist (float): the LiDAR perceived distance to the right wall
Logic:
    If the distance is above some constant threshold (defined at imports), then that's the correct direction 
'''
def decide_turn(left_dist, right_dist) -> float:
    ...
    
'''
Purpose: This function's goal is to determine the yaw that needs to be used based on the direction
Keep in mind that yaw increases in the counter clockwise direction
Parameters:
    decision (Direction): This is the direction chosen, this is in the form of Direction.LEFT, or Direction.RIGHT
    current_yaw (float): This is the current yaw being tracked on the drone
'''
def determine_yaw(decision, current_yaw) -> float:
    ...