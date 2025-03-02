from PlatformTrajectory import PlatformTrajectory
import numpy as np
from StewartPlatform import StewartPlatform
from mpl_toolkits.mplot3d import Axes3D

# Create a trajectory generator
trajectory_generator = PlatformTrajectory(
    v_apoyo=100, v_oscilacion=50, z_max=50, theta_max=15,
    y_max=10, roll_max=5, pitch_max=5, T=1, num_cycles=3
)


# Generate trajectories
steps = 300  # Total steps
dt = 0.01  # Time interval

# Offset the platforms for side-by-side placement
offset = 25  # Distance between the two platforms along the Y-axis
# Create object instances for two platforms
platform1 = StewartPlatform(r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, -offset, 0])
platform2 = StewartPlatform(r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, offset, 0])

# Generate synchronized trajectories
positions1, orientations1, positions2, orientations2 = trajectory_generator.generate_trajectory_from_data(steps,platform1, platform2)

# Visualize one component
import matplotlib.pyplot as plt

time = np.linspace(0, steps * dt, steps)
"""
# Plot X positions
plt.figure(figsize=(10, 6))
plt.plot(time, positions1[:, 0], label='Platform 1 - X')
plt.plot(time, positions2[:, 0], label='Platform 2 - X', linestyle='--')
plt.title('X Position of Platforms')
plt.xlabel('Time (s)')
plt.ylabel('X Position (cm)')
plt.legend()
plt.grid()
plt.show()

# Plot Z positions
plt.figure(figsize=(10, 6))
plt.plot(time, positions1[:, 2], label='Platform 1 - Z')
plt.plot(time, positions2[:, 2], label='Platform 2 - Z', linestyle='--')
plt.title('Z Position of Platforms')
plt.xlabel('Time (s)')
plt.ylabel('Z Position (cm)')
plt.legend()
plt.grid()
plt.show()
"""

# Plot Yaw (Theta)
plt.figure(figsize=(10, 6))
plt.plot(time, orientations1[:, 1], label='Platform 1 - Pitch (Theta)')
plt.plot(time, orientations2[:, 1], label='Platform 2 - Pitch (Theta)', linestyle='--')
plt.title('Pitch (Theta) of Platforms')
plt.xlabel('Time (s)')
plt.ylabel('Pitch (degrees)')
plt.legend()
plt.grid()
plt.show()

# Extract Roll, Pitch, and Yaw for both platforms
[Roll1, Pitch1, Yaw1] = orientations1[:, 0], orientations1[:, 1], orientations1[:, 2]
[Roll2, Pitch2, Yaw2] = orientations2[:, 0], orientations2[:, 1], orientations2[:, 2]

# Create 3D Figure
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot Platform 1's angles
ax.plot(time, Roll1, Pitch1, label="Platform 1", color="blue")
ax.plot(time, Roll2, Pitch2, label="Platform 2", color="red", linestyle="dashed")

# Labels and title
ax.set_xlabel("Time (s)")
ax.set_ylabel("Roll (degrees)")
ax.set_zlabel("Pitch (degrees)")
ax.set_title("Roll-Pitch-Yaw Evolution Over Time")

# Add legend
ax.legend()
plt.show()