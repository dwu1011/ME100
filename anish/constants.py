class Altitude_Constants:
    alpha_alt = 0.5
    target_alt = 10
    altitude_pid_gains = (3.8,0.3,3)

alpha_lidar = 0.5
distance_threshold = 10


class Pitch_Constants:
    target_front_dist = 10
    pitch_pid_gains = (5/30, 0.0 , 0.02)

class Roll_Constants:
    target_right_dist = 7.5
    roll_pid_gains = (0.9, 0.0, 0.65)