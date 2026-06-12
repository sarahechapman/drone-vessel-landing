# Drone Landing on Moving Vessel

Last summer, sailing in the Balearic Islands, a crew member tried to land a drone back onto the boat. Watching him struggle with it fascinated me: the deck was moving, the wind was unpredictable, and the drone had no idea. I wanted to understand the control problem properly, so I built this simulation.
This project models a drone autonomously landing on a moving vessel in open water. It's built in Python with a clean OOP architecture, and explores PID control, state estimation, and robustness analysis across increasingly challenging conditions.

## Architecture

The simulation is built around four core classes:

**Drone** — tracks the physical state of the drone including position, velocity, 
and mass. Can be queried for its current position or updated with a force vector 
and timestep to advance its state.

**Vessel** — tracks the position of the ship deck. Its vertical position evolves 
as a sine wave to simulate ocean swell, while remaining fixed in x and y.

**PIDController** — implements a PID control loop to compute the thrust force 
needed to drive the drone toward a target position. Inherits from a generic 
Controller base class, allowing alternative controllers (e.g. LQR) to be 
swapped in without changing the rest of the system.

**Simulation** — orchestrates everything. Steps forward in discrete time 
increments (dt), calling each component to update in the correct order and 
recording position history for analysis.

## Setup

Clone the repository:
```bash
git clone https://github.com/sarahechapman/drone-vessel-landing
cd drone-vessel-landing
```

Create and activate a virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
```

Install dependencies:
```bash
pip install -r requirements.txt
```

Run the simulation:
```bash
python3 main.py
```

Output is saved to `output.png`.

## Roadmap

- [x] Layer 1: PID control with gravity feedforward
- [ ] Layer 2: LQR optimal controller
- [ ] Layer 3: Kalman filter state estimation from noisy sensors
- [ ] Layer 4: GPS-denied navigation with IMU dead reckoning
- [ ] Layer 5: Monte Carlo robustness analysis