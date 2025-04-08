from PlatformTrajectory import PlatformTrajectory
import numpy as np
from StewartPlatform import StewartPlatform
from mpl_toolkits.mplot3d import Axes3D
# Visualize one component
import matplotlib.pyplot as plt

def plot_trajectory_summary(time, positions1, orientations1, positions2, orientations2):
    """
    Summary of the platform trajectories: 3D position, orientation over time
    :param time:
    :param positions1:
    :param orientations1:
    :param positions2:
    :param orientations2:
    :return:
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))

    # Plot X displacements
    axs[0].plot(time, positions1[:, 0], label='Platform 1 - X')
    axs[0].plot(time, positions2[:, 0], label='Platform 2 - X')
    axs[0].set_title('X Position of Platforms')
    #axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('X Position (cm)')
    axs[0].legend()
    axs[0].grid()
    # Plot Y displacements
    axs[1].plot(time, positions1[:, 1], label='Platform 1 - Y')
    axs[1].plot(time, positions2[:, 1], label='Platform 2 - Y')
    axs[1].set_title('Y Position of Platforms')
    #axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Y Position (cm)')
    axs[1].legend()
    axs[1].grid()
    # Plot Z displacements
    axs[2].plot(time, positions1[:, 2], label='Platform 1 - Z')
    axs[2].plot(time, positions2[:, 2], label='Platform 2 - Z')
    axs[2].set_title('Z Position of Platforms')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Z Position (cm)')
    axs[2].legend()
    axs[2].grid()

    # Adjust layout for better visualization
    plt.tight_layout()
    plt.show()

    # Plot roll, pitch, yaw
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


def plot_desired_lengths(platform, positions, orientations, time, label='Platform'):
    """
    Plot desired actuator lengths for a given trajectory and platform.

    :param platform:
    :param positions:
    :param orientations:
    :param time:
    :param label:
    :return:
    """
    steps = len(time)
    desired_lengths = np.zeros((steps, 6))

    for i in range(steps):
        desired_lengths[i, :] = platform.inverse_kinematics(positions[i], orientations[i])

    plt.figure(figsize=(12, 6))
    for i in range(6):
        plt.plot(time, desired_lengths[:, i], label=f'{label} Actuator {i+1}')

    plt.title(f'Desired Actuator lengths {label}')
    plt.xlabel('Time (s)')
    plt.ylabel('Length (cm)')
    plt.grid()
    plt.legend()
    plt.tight_layout()
    plt.show()


# Create a trajectory generator
#trajectory_generator = PlatformTrajectory(v_apoyo=100, v_oscilacion=50, z_max=50, theta_max=15,y_max=10, roll_max=5, pitch_max=5, T=1, num_cycles=3)
trajectory_generator = PlatformTrajectory(v_apoyo=50, v_oscilacion=20, z_max=30, theta_max=0.5, y_max=10, roll_max=10, pitch_max=10, T=1, num_cycles=3)

# Generate trajectories
steps = 300  # Total steps
dt = 0.01  # Time interval

# Offset the platforms for side-by-side placement
offset = 25  # Distance between the two platforms along the Y-axis
# Create object instances for two platforms
platform1 = StewartPlatform(0, r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, -offset, 0])
platform2 = StewartPlatform(1, r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, offset, 0])

# Generate synchronized trajectories
positions1, orientations1, positions2, orientations2 = trajectory_generator.generate_trajectory2(steps, dt, platform1, platform2)
#positions1, orientations1, positions2, orientations2 = trajectory_generator.generate_orientation_test_trajectory(steps, dt, platform1, platform2)

time = np.linspace(0, steps * dt, steps)

# --- Plot trajectory summary ---
plot_trajectory_summary(time, positions1, orientations1, positions2, orientations2)
# Plot desired actuator lengths
plot_desired_lengths(platform1, positions1, orientations1, time, label='Platform 1')
plot_desired_lengths(platform2, positions2, orientations2, time, label='Platform 2')