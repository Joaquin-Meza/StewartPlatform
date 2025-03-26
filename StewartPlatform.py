import numpy as np
import matplotlib.pyplot as plt
import serial
import time
from Controllers import DSTA
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class StewartPlatform:
    def __init__(self, id, r_B, r_P, gamma_B_deg, gamma_P_deg, serial_port=None, control_parameters=None, dt=0.01, controller_type="PID", base_offset=None, home_position=None,home_orientation=None):

        self.platform_id = id
        self.dt = dt
        self.controller_type = controller_type  # "PID" or "DSTA"
        self.kp = [100, 100, 100, 100, 100, 100]    # Individual Proportional gains for each motor
        self.ki = [50, 5, 5, 5, 5, 5]   # Individual Integral gains for each motor
        self.kd = [80, 50, 0.5, 50, 50, 50]   # Individual Derivative gains for each motor
        self.previous_errors = np.zeros(6)
        self.integral_errors = np.zeros(6)      # To store cumulative errors for integral control
        self.current_lengths = np.zeros(6)
        self.serial_port = serial.Serial(serial_port, 250000) if serial_port else None

        # Set home position and orientation
        if home_position is None:
            self.home_position = np.array([0, 0, 0.01])     # (x,y,z) Translation
        else:
            self.home_position = np.array(home_position)
        if home_orientation is None:
            self.home_orientation = np.array([0, 0, 0])  # degrees (roll, pitch, yaw)
        else:
            self.home_orientation = np.array(home_orientation)
        #home_lengths = self.inverse_kinematics(self.home_position, self.home_orientation)
        self.min_length = self.home_position[2]+2
        self.max_length = 70

        if self.serial_port:
            time.sleep(3)  # Wait for the serial connection to establish
            print('Serial communication enabled')

        self.base_points = None
        self.platform_points = None
        self.base_offset = np.array(base_offset) if base_offset else np.array([0, 0, 0])  # Default is no offset

        self.dsta_controllers = [None]*6
        # self.dsta_controllers = [DSTA(dt, 50, 100, 0.5, 0.5, w1=0.5, w2=0.9) for _ in range(6)]     # Initialize DSTA conrollers
        # (tau, k1, k2, rho1, rho2, w1=0, w2=0)
        self.dsta_controllers[0] = DSTA(dt, 50, 2500, 0.1, 0.905, w1=50, w2=70)
        self.dsta_controllers[1] = DSTA(dt, 50, 2500, 0.1, 0.905, w1=50, w2=70)
        self.dsta_controllers[2] = DSTA(dt, 50, 2500, 0.01, 0.905, w1=50, w2=70)
        self.dsta_controllers[3] = DSTA(dt, 35, 2500, 0.01, 0.905, w1=5, w2=50)
        self.dsta_controllers[4] = DSTA(dt, 35, 2500, 0.01, 0.905, w1=50, w2=70)
        self.dsta_controllers[5] = DSTA(dt, 70, 2700, 0.09, 0.905, w1=50, w2=100)


        self.DefineConfiguration(r_B, r_P, gamma_B_deg, gamma_P_deg)

        self.log = {
            'time': [],
            'desired_lengths': [],
            'current_lengths': [],
            'errors': [],
            'control_signals': [],
            'positions': [],
            'orientation': []
        }

    def DefineConfiguration(self, r_B, r_P, gamma_B_deg, gamma_P_deg):
        pi = np.pi
        #r_B, r_P = 50, 15               # Dimensions in cm
        #gamma_P_deg = 25.25 gamma_P_deg = 21.85
        gamma_B = gamma_B_deg * pi / 180      # Angles in degrees converted to radians
        gamma_P = gamma_P_deg * pi / 180      # Angles in degrees converted to radians

        # Define the geometry of the platform
        # Psi_B - polar coordinates
        psi_B = np.array([
            -gamma_B,
            gamma_B,
            2 * pi / 3 - gamma_B,
            2 * pi / 3 + gamma_B,
            4 * pi / 3 - gamma_B,
            4 * pi / 3 + gamma_B
        ])
        # psi_P (Polar coordinates)
        psi_P = ([
            5 * pi / 3 + gamma_P,
            pi / 3 - gamma_P,
            pi / 3 + gamma_P,
            3 * pi / 3 - gamma_P,
            3 * pi / 3 + gamma_P,
            5 * pi / 3 - gamma_P
        ])
        # Calculate the Cartesian coordinates of the anchor points
        self.base_points = r_B*np.array([
            [np.cos(psi_B[i]), np.sin(psi_B[i]), 0] for i in range(6)
        ]) + self.base_offset   # Apply base offset
        self.platform_points = r_P * np.array([
            [np.cos(psi_P[i]), np.sin(psi_P[i]), 0] for i in range(6)
        ])


    def rotation_matrix(self, euler_angles):
        """ Generate a rotation matrix taking as input Euler angles (roll,pitch, yaw)"""
        roll, pitch, yaw = euler_angles
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        #self.R = Rz @ Ry @ Rx
        return Rx @ Ry @ Rz

    def inverse_kinematics(self, position, orientation):
        """
        Compute the inverse kinematics of a Stewart Platform and enforce actuator limits
        :param base_points: (ndarray) Coordinates of anchors for B_i (6x3)
        :param platform_points: (ndarray) Coordinates of anchors of P_i (6x3)
        :param position: (ndarray) Translation vector [x,y,z] of platform center
        :param orientation: (ndarray) Euler Angles [roll, pitch, yaw] of platform
        :return: lengths: (ndarray) Actuator lengths (6x1)
        """

        # Apply constraints to prevent excessive rotation
        orientation[0] = np.clip(orientation[0], -10, 10)       # Roll constraint
        orientation[1] = np.clip(orientation[1], -20, 20)       # Pitch constraint
        orientation[2] = np.clip(orientation[2], -10, 10)       # Yaw constraint

        R = self.rotation_matrix(orientation)       # Rotation matrix from euler angles

        P_global = np.array([R @ p + position for p in self.platform_points])   # Global position of points P_i
        lengths = np.linalg.norm(P_global - self.base_points, axis=1)           # Compute the lengths of each actuator:

        # Convert from simulation (40 cm min) to encoder readings (0 cm min)
        # Enforce actuator constraints
        lengths = np.clip(lengths, self.min_length, self.max_length)    # Keep within actuator limits
        #           y = (x-40)+5
        lengths = lengths-0     # Scale to 0 - 30 range
        return lengths
    

    def pid_control(self, desired_length, actuator_idx):
        """
        PID Controller to adjust actuator's lengths
        :param desired_lengths: (ndarray) Desired lengths to reach the desired configuration
        :param current_lengths: (ndarray) Actual position of the actuators (retrieved through serial)
        :param previous_errors: (ndarray) Error from the previous iteration (For the Integral component)
        :param kp: (float) Proportional Gain
        :param ki: (float) Integral Gain
        :param kd: (float) Derivative Gain
        :param dt: (float) Time interval for each step
        :return control_signals: (ndarray) Control signals to move each actuator
        :return errors: (ndarray) Current errors
        """
        # Determine the actual error
        error = desired_length - self.current_lengths[actuator_idx]

        # Proportional component
        P = self.kp[actuator_idx] * error

        # Integral Component
        self.integral_errors[actuator_idx] += error * self.dt
        # I = self.ki * (self.previous_errors + errors * self.dt)
        I = self.ki[actuator_idx] * self.integral_errors[actuator_idx]

        # Derivative component
        D = self.kd[actuator_idx] * ((error - self.previous_errors[actuator_idx]) / self.dt)     # Derivative component

        # Compute total control signal
        control_signals = P + I + D                         # Control signals

        # Update previous error
        self.previous_errors[actuator_idx] = error

        # Clamp control signals within actuator limits
        new_length = self.current_lengths[actuator_idx] + control_signals*self.dt
        new_length = np.clip(new_length, self.min_length, self.max_length)
        control_correction = new_length-self.current_lengths[actuator_idx]
        return control_correction

    def dsta_control(self, desired_lengths):
        """
        DSTA Controller with individual control for each motor
        :param desired_lengths:
        :return control_signals:
        """
        control_signals = np.zeros(6)

        for i in range(6):
            control_signals[i] = self.dsta_controllers[i].derivative(desired_lengths[i]-self.current_lengths[i])

        # Clamp control signals within actuator limits
        #new_lengths = self.current_lengths + control_signals * self.dt
        #new_lengths = np.clip(new_lengths, self.min_length, self.max_length)
        # new_lengths - self.current_lengths
        return control_signals

    def send_control_signal(self, control_signals):
        if self.serial_port:
            signal_str = ','.join(f'{signal:.2F}'for signal in control_signals)
            self.serial_port.write(signal_str.encode())

    def read_encoder_feedback(self):
        if self.serial_port and self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode().strip()
            encoder_values = np.array([float(value) for value in data.split(',')])
            return encoder_values
        return None

    def visualize_platform(self, position, orientation, ax):
        ax.clear()
        R = self.rotation_matrix(orientation)
        P_global = np.array([R @ p + position for p in self.platform_points])

        ax.scatter(self.base_points[:, 0], self.base_points[:, 1], self.base_points[:, 2], c='red', label='Base Points')
        ax.scatter(P_global[:, 0], P_global[:, 1], P_global[:, 2], c='blue', label='Platform Points')
        # Draw legs connecting base and platform
        for i in range(6):
            ax.plot([self.base_points[i, 0], P_global[i, 0]],
                    [self.base_points[i, 1], P_global[i, 1]],
                    [self.base_points[i, 2], P_global[i, 2]], 'k--')

        # Define polygons for base and platform planes
        base_plane = Poly3DCollection([self.base_points[:, :3]], alpha=0.3, color='green')
        platform_plane = Poly3DCollection([P_global[:, :3]], alpha=0.3, color='blue')

        # Add planes to the plot
        ax.add_collection3d(base_plane)
        ax.add_collection3d(platform_plane)
        # Add an indicator vector showing the top direction of the platform
        top_vector_start = position  # Start from the platform center
        top_vector_end = position + R @ np.array([0, 0, 25])  # Project a vector upwards
        ax.quiver(top_vector_start[0], top_vector_start[1], top_vector_start[2],
                  top_vector_end[0] - top_vector_start[0],
                  top_vector_end[1] - top_vector_start[1],
                  top_vector_end[2] - top_vector_start[2],
                  color='purple', linewidth=2, label='Platform Top Indicator')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        # Fix axis limits to avoid constant resizing
        ax.set_xlim(-50, 50)  # Set fixed range for X-axis
        ax.set_ylim(-60, 60)  # Set fixed range for Y-axis
        ax.set_zlim(0, 75)  # Set fixed range for Z-axis
        ax.set_box_aspect([1, 1, 0.5])  # Optional: Adjust aspect ratio
        plt.pause(0.1)

    def visualize_platform2(self, position, orientation, ax):
        R = self.rotation_matrix(orientation)
        P_global = np.array([R @ p + position for p in self.platform_points])

        # Plot base points and platform points
        ax.scatter(self.base_points[:, 0], self.base_points[:, 1], self.base_points[:, 2], c='red', label='Base Points')
        ax.scatter(P_global[:, 0], P_global[:, 1], P_global[:, 2], c='blue', label='Platform Points')

        # Plot actuator lines
        for i in range(6):
            ax.plot([self.base_points[i, 0], P_global[i, 0]],
                    [self.base_points[i, 1], P_global[i, 1]],
                    [self.base_points[i, 2], P_global[i, 2]], 'k--')

        # Define polygons for base and platform planes
        base_plane = Poly3DCollection([self.base_points[:, :3]], alpha=0.3, color='green')
        platform_plane = Poly3DCollection([P_global[:, :3]], alpha=0.3, color='blue')

        # Add planes to the plot
        ax.add_collection3d(base_plane)
        ax.add_collection3d(platform_plane)
        # Add an indicator vector showing the top direction of the platform
        top_vector_start = position  # Start from the platform center
        top_vector_end = position + R @ np.array([0, 0, 25])  # Project a vector upwards
        ax.quiver(top_vector_start[0], top_vector_start[1], top_vector_start[2],
                  top_vector_end[0] - top_vector_start[0],
                  top_vector_end[1] - top_vector_start[1],
                  top_vector_end[2] - top_vector_start[2],
                  color='purple', linewidth=2, label='Platform Top Indicator')
        # Fix axis limits to avoid constant resizing
        ax.set_xlim(-50, 50)  # Set fixed range for X-axis
        ax.set_ylim(-60, 60)  # Set fixed range for Y-axis
        ax.set_zlim(0, 75)  # Set fixed range for Z-axis
        ax.set_box_aspect([1, 1, 0.5])  # Optional: Adjust aspect ratio


    def run_simulation(self, steps, position=None, orientation=None):
        "Run the simulation with the defined home position if not target position is given"

        if position is None:
            position = self.home_position
        if orientation is None:
            orientation = self.home_orietnation

        desired_lengths = self.inverse_kinematics(position, orientation)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for step in range(steps):
            control_signals = self.pid_control(desired_lengths)
            self.send_control_signal(control_signals)

            encoder_feedback = self.read_encoder_feedback()
            if encoder_feedback is not None:
                self.current_lengths = encoder_feedback

            self.visualize_platform(position, orientation)
            print(f"Step {step}: Current Lengths: {self.current_lengths}")

        plt.draw()
        if self.serial_port:
            self.serial_port.close()

    def simulate_trajectory(self, positions, orientations, steps):
        """
        Simulates the trajectory of the Stewart Platform
        :param positions: Desired positions over time (steps x 3)
        :param orientations: Desired orientation over time (steps x 3)
        :param steps: Number of simulation steps
        :return:
        """

        # Clear previous logs
        self.log = {
            'time': [],
            'desired_lengths': [],
            'current_lengths': [],
            'control_signals': []
        }

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for step in range(steps):
            position = positions[step]
            orientation = orientations[step]


            desired_lengths = self.inverse_kinematics(position, orientation)
            # control_signals = self.pid_control(desired_lengths)

            # Choose controller
            if self.controller_type == 'PID':
                control_signals = self.pid_control(desired_lengths)
            elif self.controller_type == 'DSTA':
                control_signals = self.dsta_control(desired_lengths)
            else:
                control_signals = [np.zeros(6)]

            # Handle actuator movement (with or without serial communication)
            if self.serial_port:
                try:
                    self.send_control_signal(control_signals)
                    encoder_feedback = self.read_encoder_feedback()
                    if encoder_feedback is not None:
                        self.current_lengths = encoder_feedback
                except Exception as e:
                    print(f"Serial communication error: {e}")
            else:
                self.current_lengths += control_signals * self.dt

            # Log data for plotting
            self.log['time'].append(step * self.dt)
            self.log['desired_lengths'].append(desired_lengths.copy())
            self.log['current_lengths'].append(self.current_lengths.copy())
            self.log['control_signals'].append(control_signals.copy())

            # Update visualization
            if step % 5 == 0:
                print(f"Step {step}: Desired Position: {position}, Orientation: {orientation}")
                self.visualize_platform(position, orientation, ax)

        plt.draw()
        # Call the function to plot results
        self.plot_actuator_response(self.log)

    def plot_actuator_response(self, log):
        """
            Plots the desired position against the current position of the actuators,
            along with the control values for each actuator.

            :param log: Dictionary containing simulation logs with keys:
                'time': List of time steps
                'desired_lengths': List of desired actuator lengths
                'current_lengths': List of actual actuator lengths
                'control_signals': List of control signals
            """
        time_steps = log['time']
        desired_lengths = log['desired_lengths']
        current_lengths = log['current_lengths']
        control_signals = log['control_signals']
        # Plot Desired vs Current Actuator Lengths
        fig, axs = plt.subplots(2, 1, figsize=(10, 8))
        for i in range(6):
        #i=5
            axs[0].plot(time_steps, [dl[i] for dl in desired_lengths], label=f'Desired Length {i + 1}', linestyle='--')
            axs[0].plot(time_steps, [cl[i] for cl in current_lengths], label=f'Current Length {i + 1}')

        axs[0].set_title("Desired vs Current Actuator Lengths")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Length (cm)")
        axs[0].legend()
        axs[0].grid(True)

        # Plot Control Signals
        for i in range(6):
            axs[1].plot(time_steps, [cs[i] for cs in control_signals], label=f'Control Signal {i + 1}')

        axs[1].set_title("Control Signals for Actuators")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Control Value")
        axs[1].legend()
        axs[1].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_platform_response(self, log):
        """
        Plots the desired position and orientation of the Stewart Platform against the current values
        :param log: Dictionary containing the simulation logs with keys
        :return:
        """
        time_steps = log['time']
        desired_positions = np.array(log['desired_positions'])
        current_positions = np.array(log['current_positions'])
        desired_orientations = np.array(log['desired_orientations'])
        current_orientations = np.array(log['current_orientations'])

        fig, axs = plt.subplots(2, 1, figsize=(10, 8))

        # Plot Desired vs Current Position
        labels = ['X', 'Y', 'Z']
        for i in range(3):
            axs[0].plot(time_steps, desired_positions[:, i], linestyle='--', label=f'Desired {labels[i]}')
            axs[0].plot(time_steps, current_positions[:, i], label=f'Current {labels[i]}')

        axs[0].set_title("Desired vs Current Platform Position")
        axs[0].set_xlabel("Time (s)")
        axs[0].set_ylabel("Position (mm)")
        axs[0].legend()
        axs[0].grid(True)

        # Plot Desired vs Current Orientation
        labels = ['Roll', 'Pitch', 'Yaw']
        for i in range(3):
            axs[1].plot(time_steps, desired_orientations[:, i], linestyle='--', label=f'Desired {labels[i]}')
            axs[1].plot(time_steps, current_orientations[:, i], label=f'Current {labels[i]}')

        axs[1].set_title("Desired vs Current Platform Orientation")
        axs[1].set_xlabel("Time (s)")
        axs[1].set_ylabel("Orientation (degrees)")
        axs[1].legend()
        axs[1].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_actuator_rmse(self, log):
        """
        Compute and plots the RMSE of the actuators
        :return:
        """

        desired_lengths = np.array(log['desired_lengths'])
        current_lengths = np.array(log['current_lengths'])

        # Compute RMSE for each actuator
        rmse = np.sqrt(np.mean((desired_lengths-current_lengths)**2, axis=0))

        # Plot RMSE
        plt.figure(figsize=(8, 5))
        plt.bar(range(1, 7), rmse,tick_label=[f'Actuator {i+1}' for i in range(6)])
        plt.title("RMSE of Actuator position")
        plt.xlabel("Actuator")
        plt.ylabel("RMSE (cm)")
        plt.grid(axis='y')
        plt.show()


    def save_log(self, filename="simulation_log.json"):
        import json
        with open(filename, "w") as file:
            json.dump(self.log, file, indent=4)
            print(f"Log saved to {filename}")

    def control_bounds(self, x):
        if x >= 255:
            x = 255
        elif x <= -255:
            x = -255
        return x+255

    # Define a desired trajectory for the platform
    def generate_trajectory(self, steps, dt):
        """
        Generates a trajectory for the platform.

        Parameters:
            steps (int): Number of steps in the trajectory.
            dt (float): Time interval between steps.

        Returns:
            positions (ndarray): Desired positions over time (steps x 3).
            orientations (ndarray): Desired orientations over time (steps x 3).
        """
        t = np.linspace(0, steps * dt, steps)

        # Compute min and max actuator lengths
        min_length = np.linalg.norm(self.inverse_kinematics(self.home_position, self.home_orientation)) + 5
        max_length = min_length + 25

        positions = np.array([
            0.5 * np.sin(0.5 * t),  # X position (sinusoidal)
            0.5 * np.cos(0.5 * t),  # Y position (cosine)
            np.clip(  # Ensure Z position is within min/max limits
                self.home_position[2] + 0.2 * np.sin(0.2 * t),
                min_length,
                max_length
            )
        ]).T
        orientations = np.array([
            0.1 * np.sin(0.1 * t),  # Roll
            0.1 * np.sin(0.2 * t),  # Pitch
            0.1 * np.cos(0.1 * t)  # Yaw
        ]).T
        return positions, orientations

    def run_trial(self, positions, orientations, steps):
        """
        Simulates the trajectory of the Stewart Platform with
        :param positions: (ndarray) Desired positions over time (steps x 3)
        :param orientations: (ndarray) Desired orientation over time (steps x 3)
        :param steps: (ndarray) Number of simulation steps
        :return:
        """
        self.log = {
            'time': [],
            'desired_lengths': [],
            'current_lengths': [],
            'control_signals': [],
        }
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Initialize actuator lengths at home position
        self.current_lengths = self.inverse_kinematics(self.home_position, self.home_orientation)

        for step in range(steps):
            position = positions[step]
            orientation = orientations[step]

            # Compute desired lengths based on desired position and orientation
            desired_lengths = self.inverse_kinematics(position, orientation)
            # Compute control signals for each actuator
            errors = desired_lengths - self.current_lengths

            control_signals = np.zeros(6)

            for i in range(6):
                if self.controller_type == 'PID':
                    control_signals[i] = self.pid_control(desired_lengths[i], i)
                if self.controller_type == 'DSTA':
                    control_signals[i] = self.dsta_controllers[i].derivative(errors[i])  # Individual actuator control update

            # Handle actuator movement (with or without serial communication)
            if self.serial_port:
                try:
                    self.send_control_signal(control_signals)
                    encoder_feedback = self.read_encoder_feedback()
                    if encoder_feedback is not None:
                        self.current_lengths = encoder_feedback
                except Exception as e:
                    print(f"Serial communication error: {e}")
            else:
                self.current_lengths += control_signals * self.dt
                self.current_lengths = np.clip(self.current_lengths, self.min_length, self.max_length)

            # Update visualization
            #if step % 5 == 0:
                #print(f"Step {step}: Desired Position: {position}, Orientation: {orientation}")
                #self.visualize_platform(position, orientation, ax)

            #plt.draw()
            # Log data for plotting
            self.log['time'].append(step*self.dt)
            self.log['desired_lengths'].append(desired_lengths.copy())
            self.log['current_lengths'].append(self.current_lengths.copy())
            self.log['control_signals'].append(control_signals.copy())
            plt.draw()

        self.plot_actuator_response(self.log)

    def test(self, positions, orientations, steps):

        self.log = {
            'time': [],
            'desired_lengths': [],
            'current_lengths': [],
            'control_signals': []
        }

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Initialize actuator lengths at home position
        self.current_lengths = self.inverse_kinematics(self.home_position, self.home_orientation)

        for step in range(steps):
            position = positions[step]
            orientation = orientations[step]

            # Compute desired lengths based on desired position and orientation
            desired_lengths = self.inverse_kinematics(position, orientation)
            # Compute control signals for each actuator
            errors = desired_lengths - self.current_lengths             # Explicit error calculation for each actuator

            control_signals = np.zeros(6)
            for i in range(6):
                if self.controller_type == 'PID':
                    control_signals[i] = self.pid_control(desired_lengths[i], i)
                if self.controller_type == 'DSTA':
                    control_signals[i] = self.dsta_controllers[i].derivative(errors[i]) # Individual actuator control update

            # Apply control signals to actuators
            self.current_lengths += control_signals * self.dt
            self.current_lengths = np.clip(self.current_lengths, self.min_length, self.max_length)

            # Log the results
            self.log['time'].append(step * self.dt)
            self.log['desired_lengths'].append(desired_lengths.copy())
            self.log['current_lengths'].append(self.current_lengths.copy())
            self.log['control_signals'].append(control_signals.copy())

            # Update visualization
            if step % 5 == 0:
                print(f"Step {step}: Desired Position: {position}, Orientation: {orientation}")
                self.visualize_platform(position, orientation, ax)

        plt.draw()

        #self.plot_actuator_response(self.log)  # Plot the results
        #self.plot_actuator_rmse()
