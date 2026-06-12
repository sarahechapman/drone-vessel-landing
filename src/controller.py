from abc import ABC, abstractmethod
from scipy.linalg import solve_continuous_are
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

class LQRController(Controller):
    def __init__(self, Q, R, mass=1.0):
        self.Q = Q
        self.R = R
        A = np.array([[0, 1],[0, 0]])
        B = np.array([[0],[1/mass]])
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.inv(R) @ B.T @ P
    
    def compute(self, target_position, current_position, current_velocity, dt):
        current_state = np.array([current_position, current_velocity])
        target_state = np.array([target_position, 0])
        error = np.array(current_state) - np.array(target_state)
        force = -self.K @ error
        return float(force)
