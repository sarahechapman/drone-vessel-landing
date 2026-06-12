import numpy as np

class Vessel:
    def __init__(self, x=0.0, y=0.0, z=0.0, amplitude=3.0, frequency=0.5, phase=0):
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = phase
        self.position = np.array([x, y, z], dtype=float)

    def update(self, t):
        self.position[2] = self.amplitude * np.sin(self.frequency * t + self.phase)

    def get_position(self):
        return self.position.copy()