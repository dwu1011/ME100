import math

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
