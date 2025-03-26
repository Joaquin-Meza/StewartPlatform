from PlatformTrajectory import PlatformTrajectory
import numpy as np
from StewartPlatform import StewartPlatform
from mpl_toolkits.mplot3d import Axes3D
# Visualize one component
import matplotlib.pyplot as plt

# Create a trajectory generator
#trajectory_generator = PlatformTrajectory(v_apoyo=100, v_oscilacion=50, z_max=50, theta_max=15,y_max=10, roll_max=5, pitch_max=5, T=1, num_cycles=3)
trajectory_generator = PlatformTrajectory(v_apoyo=50, v_oscilacion=20, z_max=50, theta_max=0.5, y_max=10, roll_max=5, pitch_max=10, T=3, num_cycles=1)

# Generate trajectories
steps = 300  # Total steps
dt = 0.01  # Time interval

# Offset the platforms for side-by-side placement
offset = 25  # Distance between the two platforms along the Y-axis
# Create object instances for two platforms
platform1 = StewartPlatform(0, r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, -offset, 0])
platform2 = StewartPlatform(1, r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, offset, 0])

# Generate synchronized trajectories
#positions1, orientations1, positions2, orientations2 = trajectory_generator.generate_trajectory(steps, dt, platform1, platform2)
positions1, orientations1, positions2, orientations2 = trajectory_generator.generate_orientation_test_trajectory(steps, dt, platform1, platform2)

time = np.linspace(0, steps * dt, steps)

# Create a figure with subplots
fig, axs = plt.subplots(3, 1, figsize=(10, 12))

# Plot X positions
axs[0].plot(time, positions1[:, 0], label='Platform 1 - X')
axs[0].plot(time, positions2[:, 0], label='Platform 2 - X', linestyle='--')
axs[0].set_title('X Position of Platforms')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('X Position (cm)')
axs[0].legend()
axs[0].grid()

# Plot Z positions
axs[1].plot(time, positions1[:, 2], label='Platform 1 - Z')
axs[1].plot(time, positions2[:, 2], label='Platform 2 - Z', linestyle='--')
axs[1].set_title('Z Position of Platforms')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Z Position (cm)')
axs[1].legend()
axs[1].grid()

# Plot Yaw (Theta)
axs[2].plot(time, orientations1[:, 1], label='Platform 1 - Pitch (Theta)')
axs[2].plot(time, orientations2[:, 1], label='Platform 2 - Pitch (Theta)', linestyle='--')
axs[2].set_title('Pitch (Theta) of Platforms')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Pitch (degrees)')
axs[2].legend()
axs[2].grid()

# Adjust layout for better visualization
plt.tight_layout()

# Show the figure
plt.show()

# Create a separate 3D plot
fig3d = plt.figure(figsize=(10, 7))
ax = fig3d.add_subplot(111, projection='3d')

# Extract Roll, Pitch, and Yaw for both platforms
Roll1, Pitch1, Yaw1 = orientations1[:, 0], orientations1[:, 1], orientations1[:, 2]
Roll2, Pitch2, Yaw2 = orientations2[:, 0], orientations2[:, 1], orientations2[:, 2]

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

# Show the 3D figure
plt.show()