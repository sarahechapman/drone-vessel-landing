from abc import ABC, abstractmethod
import numpy as np

class Controller(ABC):
    
    @abstractmethod
    def compute(self, current, target, dt):
        pass

class PIDController(Controller):
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.initialised = False

    def compute(self, target_position, current_position, dt):
        if not self.initialised:
            self.previous_error = target_position - current_position
            self.initialised = True
        error = target_position-current_position
        force = self.Kp * error
        force += self.Kd * (error - self.previous_error) / dt
        self.integral += error
        force += self.Ki * self.integral * dt
        self.previous_error = error
        return force
