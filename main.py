import matplotlib.pyplot as plt
from src.simulation import Simulation

sim = Simulation(dt=0.01)
sim.run(duration=10.0)

plt.figure(figsize=(10, 6))
plt.plot(sim.time_history, sim.drone_position_history, label='Drone')
plt.plot(sim.time_history, sim.vessel_position_history, label='Vessel')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.title('Drone landing on moving vessel')
plt.legend()
plt.grid(True)
plt.savefig('output.png')
print("Done - saved to output.png")