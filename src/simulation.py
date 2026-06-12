import numpy as np
from src.drone import Drone
from src.vessel import Vessel
from src.controller import PIDController, LQRController

class Simulation:
    def __init__(self, dt=0.01):
        self.drone = Drone()
        self.lqr_drone = Drone()
        self.vessel = Vessel()
        self.controller = PIDController(Kp=15, Ki=0.5, Kd=8)
        self.dt = dt
        self.t = 0.0
        self.time_history = []
        self.drone_position_history = []
        self.vessel_position_history = []
        self.lqr = LQRController(Q=np.array([[50, 0], [0, 1]]),R=np.array([[0.1]]),mass=self.drone.mass)
        self.lqr_position_history = []

    def step(self):
        self.vessel.update(self.t)
        vessel_position = self.vessel.get_position()
        drone_position = self.drone.get_position()
        force = self.controller.compute(vessel_position[2], drone_position[2], self.dt)
        force += self.drone.mass * self.drone.g
        self.drone.update(force, self.dt)
        self.time_history.append(self.t)
        self.drone_position_history.append(drone_position[2])
        self.vessel_position_history.append(vessel_position[2])
        lqr_drone_position = self.lqr_drone.get_position()
        lqr_force = self.lqr.compute(vessel_position[2], lqr_drone_position[2],self.lqr_drone.velocity[2],self.dt)
        lqr_force += self.lqr_drone.mass * self.lqr_drone.g
        self.lqr_drone.update(lqr_force, self.dt)
        self.lqr_position_history.append(lqr_drone_position[2])
        self.t += self.dt
    
    def run(self, duration):
        while self.t < duration:
            self.step()
