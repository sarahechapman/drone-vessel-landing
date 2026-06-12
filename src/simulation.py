import numpy as np
from src.drone import Drone
from src.vessel import Vessel
from src.controller import PIDController

class Simulation:
    def __init__(self, dt=0.01):
        self.drone = Drone()
        self.vessel = Vessel()
        self.controller = PIDController(Kp=15, Ki=0.1, Kd=8)
        self.dt = dt
        self.t = 0.0
        self.time_history = []
        self.drone_position_history = []
        self.vessel_position_history = []

    def step(self):
        self.vessel.update(self.t)
        vessel_position = self.vessel.get_position()
        drone_position = self.drone.get_position()
        force = self.controller.compute(vessel_position[2], drone_position[2], self.dt)
        self.drone.update(force, self.dt)
        self.time_history.append(self.t)
        self.drone_position_history.append(drone_position[2])
        self.vessel_position_history.append(vessel_position[2])
        self.t += self.dt
    
    def run(self, duration):
        while self.t < duration:
            self.step()
