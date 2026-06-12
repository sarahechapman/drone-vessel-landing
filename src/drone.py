import numpy as np

class Drone:
    def __init__(self, x=0.0, y=0.0, z=10.0):
        self.position = np.array([x, y, z], dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=float)
        self.mass = 1.0  # kg
        self.g = 9.81    # m/s^2

    def update(self, force, dt):
        acceleration = force / self.mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

    def get_position(self):
        return self.position.copy()