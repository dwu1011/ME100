import math

class PID:
    def __init__ (self, KP, KI, KD, dt):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.dt = dt
        self.lastErr = 0
        self.totalErr = 0

    def calculate(self, current, setpoint):
        err = setpoint - current

        self.totalErr += err * self.dt

        derivedErr = (err - lastErr)/self.dt

        output = self.KP * err + self.KI * self.totalErr + self.KD * derivedErr

        lastErr = err

        return self.clamp(output, 0 ,1)
    
    def calculate(self, measurement):
        return self.calculate(measurement, 0)
    
    def clamp(self, value, min, max):
        return math.max(min, math.min(value, max))