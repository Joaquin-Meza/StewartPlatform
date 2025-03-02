import numpy as np
import matplotlib.pyplot as plt

def generate_gait_trajectory(steps=1, step_length=0.8, step_height=0.05, dt=0.01):
    """
    Generates a trajectory for the foot soles during a gait cycle.

    Parameters:
        steps (int): Number of gait cycles.
        step_length (float): Length of one step (meters).
        step_height (float): Maximum height of foot clearance (meters).
        dt (float): Time step for trajectory generation.

    Returns:
        time (ndarray): Time array.
        X (ndarray): Forward movement trajectory.
        Z (ndarray): Vertical foot clearance trajectory.
    """
    T = 1.0  # Total duration of one gait cycle (assume 1s per step)
    stance_ratio = 0.6  # 60% stance, 40% swing

    t_stance = np.linspace(0, stance_ratio * T, int(stance_ratio * T / dt))
    t_swing = np.linspace(0, (1 - stance_ratio) * T, int((1 - stance_ratio) * T / dt))

    # X-axis (horizontal forward motion)
    X_stance = np.linspace(0, step_length / 2, len(t_stance))  # Foot is on ground
    X_swing = np.linspace(step_length / 2, step_length, len(t_swing))  # Foot moves forward

    # Z-axis (vertical clearance)
    Z_stance = np.zeros(len(t_stance))  # Foot on ground
    Z_swing = step_height * (1 - np.cos(np.pi * np.linspace(0, 1, len(t_swing))))  # Parabolic swing phase

    # Combine stance and swing phases
    X = np.concatenate([X_stance, X_swing])
    Z = np.concatenate([Z_stance, Z_swing])
    time = np.linspace(0, T * steps, len(X))

    return time, X, Z

# Generate gait cycle trajectory
time, X, Z = generate_gait_trajectory(steps=6)

# Plot the trajectory
plt.figure(figsize=(10, 5))
plt.plot(X, Z, marker="o", label="Foot Trajectory")
plt.xlabel("Forward Motion (m)")
plt.ylabel("Vertical Motion (m)")
plt.title("Foot Trajectory During Gait Cycle")
plt.legend()
plt.grid()
plt.show()
