class Altitude_Constants:
    alpha_alt = 0.7
    target_alt = 10
    altitude_pid_gains = (4.5,0.24,3.1) #4.5/0/3

alpha_lidar = 0.3
distance_threshold = 10


class Pitch_Constants:
    target_front_dist = 10
    pitch_pid_gains = (0.6, 0.0 , 0.75)
    # pitch_pid_gains = (5/30, 0, 0.02)

class Roll_Constants:
    target_right_dist = 5
    roll_pid_gains = (0.5, 0.0, 0.65)
    
