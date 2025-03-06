import time
from main2 import simulate_dual_platforms, test_Actuator
import serial
from StewartPlatform import StewartPlatform
from PlatformTrajectory import PlatformTrajectory
import json

# Initialize trajectory generator
trajectory_generator = PlatformTrajectory(v_apoyo=50, v_oscilacion=10, z_max=50, theta_max=15,y_max=10, roll_max=5, pitch_max=5, T=5, num_cycles=3)

# Simulation parameters
dt = 0.1  # Time step
expected_time = 30  # Total simulation time
steps = int(expected_time / dt)  # Total steps

# Offset the platforms for side-by-side placement
offset = 25  # Distance between the two platforms along the Y-axis
# Create object instances for two platforms

platform1 = StewartPlatform(0, r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, -offset, 0])
platform2 = StewartPlatform(1, r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, offset, 0])

# Generate synchronized trajectories
positions1, orientations1, positions2, orientations2 = trajectory_generator.generate_trajectory(steps, dt, platform1, platform2)


positions1[:, 1] -= offset  # Left platform (negative Y direction)
positions2[:, 1] += offset  # Right platform (positive Y direction)

# Parameters
dt = 0.05
steps = len(positions1)  # Ensure steps match the trajectory length

# Begin main loop
# simulate_dual_platforms(platform1, platform2, positions1, orientations1, positions2, orientations2, steps, dt=0.1, arduino=None, update_interval=5, ideal_simulation=0)

simulate_dual_platforms(
                    platform1=platform1,
                    platform2=platform2,
                    positions1=positions1,
                    orientations1=orientations1,
                    positions2=positions2,
                    orientations2=orientations2,
                    steps=steps,
                    dt=dt,
                    update_interval=5,  # Update visualization every 5 steps
                    arduino=None,
                    ideal_simulation=1
                )