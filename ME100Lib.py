from enum import Enum

import E100_functions

import numpy as np

decision_thresh = 12 # Threshold to determine a turn

'''
Purpose: This class's goal is to operate a basic PID algorithm
Parameters:
    KP (float): the tuned P gain
    KI (float): the tuned I gain
    KD (float): the tuned D gain
    dt (float): the expected delta time between runs of calculate()
Methods:
    calculate: given the current and setpoint positions, determine a PID calculated output
    set_p: set the p gain internally, used for gain scheduling
'''
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
    
    def set_p(self, p: float):
        self.KP = p
    

'''
Purpose: This function's goal is to apply a filter to some data input
Parameters:
    alpha (float): the tuned alpha gain for the filter
    dat (float): the new datum provided by the sensor
    prev (float): the previous datum
'''
def find_lpf(alpha: float, dat: float, prev: float) -> float:
    return alpha*dat + (1-alpha)*prev

'''
Purpose: This function's goal is to take a value and clamp it between 2 values
Note: The function does NOT map, it enforces a clamp
Parameters:
    value (float): the inputted datum
    mi (float): the minimum of the range (inclusive)
    ma (float): the maximum of the range (inclusive)
'''
def clamp(value: float, mi: float, ma: float) -> float:
    return max(mi, min(value, ma))

Direction = Enum('Direction', ['LEFT', 'RIGHT', 'STRAIGHT', 'BACK', 'TOP', 'BOTTOM'])

'''
Purpose: This function's goal is to determine whether it's a good idea to turn left or right
Parameters:
    left_dist (float): the LiDAR perceived distance to the left wall
    right_dist (float): the LiDAR perceived distance to the right wall
Logic:
    If the distance is above some constant threshold (defined at imports), then that's the correct direction 
'''
def decide_turn(left_dist: float, right_dist: float) -> Direction:
    if left_dist >= decision_thresh and left_dist > right_dist:
        return Direction.LEFT
    elif right_dist >= decision_thresh and right_dist >= left_dist:
        return Direction.RIGHT
    else:
        return Direction.STRAIGHT
    
'''
Purpose: This function's goal is to determine whether it's a good idea to go up or down
Parameters:
    top_dist (float): the LiDAR perceived distance to the top wall
    bot_dist (float): the LiDAR perceived distance to the bottom wall
Logic:
    If the distance is above some constant threshold (defined at imports), then that's the correct direction 
'''
def decide_vertical(top_dist: float, bot_dist: float) -> Direction:
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
'''
def determine_yaw(decision: Direction, current_yaw: float) -> float:
    if(decision is Direction.LEFT):
        return current_yaw - 90
    elif(decision is Direction.RIGHT):
        return current_yaw + 90
    return current_yaw

'''
Purpose: This function's goal is to access the custom top lidar sensor and return the distance to it
Parameters:
    client (Client): This is the Client object connecting to the simulation
    noise_mag (float): magnitude of simulated noise on the sensor
'''
def get_custom_lidar(client,noise_mag = 1) -> float:
    lidardata_new = client.getLidarData(lidar_name="LidarSensorNew")
    points5= E100_functions.get_lidarData_points(lidardata_new)
    with E100_functions.warnings.catch_warnings():
        E100_functions.warnings.simplefilter("ignore", category=RuntimeWarning)
        new_dist= np.linalg.norm(np.mean(points5, axis=0))
    new_dist += noise_mag*E100_functions.noise_gen(new_dist)
    return new_dist
