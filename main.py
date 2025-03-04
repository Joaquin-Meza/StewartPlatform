import time

import numpy as np

from StewartPlatform import StewartPlatform
from Controllers import DSTA
import serial
import pandas as pd
import matplotlib.pyplot as plt
from PlatformTrajectory import PlatformTrajectory


# Initialize Serial Communication with microcontroller
# ser = serial.Serial('COM8', 250000)
# Initialize trajectory generator
trajectory_generator = PlatformTrajectory(v_apoyo=50, v_oscilacion=20, z_max=50, theta_max=15, y_max=10, roll_max=5, pitch_max=30, T=1, num_cycles=1)

# Create object instance of the platform / Define geometry of the platform
# Offset the platforms for side-by-side placement
offset = 25  # Distance between the two platforms along the Y-axis
# Create object instances for two platforms
stewart = StewartPlatform(r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port='COM5', controller_type='DSTA', base_offset=[0, -offset, 0])
time.sleep(3)

# Default parameters for simulation

dt = 0.1
expected_time = 30
steps = int(expected_time/dt)
time_vector = np.linspace(0, expected_time, steps, endpoint=False)

# Generate synchronized trajectories
positions, orientation, positions2, orientations2 = trajectory_generator.generate_trajectory(steps,dt, stewart, stewart)

#positions, orientation = trajectory_generator.generate_orientation_test_trajectory(steps)

#stewart.simulate_trajectory(positions, orientation, steps)
stewart.run_trial(positions, orientation, steps)
#stewart.test(positions, orientation, steps)

