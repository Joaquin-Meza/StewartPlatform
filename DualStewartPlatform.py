import numpy as np
import matplotlib.pyplot as plt

class DualStewartPlatform:
    def __init__(self, platform1, platform2, dt=0.01):
        """
        Initializes the dual Stewart platform system.

        Parameters:
            platform1 (StewartPlatform): First Stewart platform.
            platform2 (StewartPlatform): Second Stewart platform.
            dt (float): Time step for the simulation.
        """
        self.platform1 = platform1
        self.platform2 = platform2
        self.dt = dt

    def generate_dual_trajectory(self, steps, dt, phase_shift=0.5):
        """
        Generates synchronized trajectories for both platforms.

        Parameters:
            steps (int): Number of steps in the trajectory.
            dt (float): Time interval between steps.
            phase_shift (float): Phase shift between the two platforms (in fraction of the cycle).

        Returns:
            positions1 (ndarray): Trajectory for platform 1 positions (steps x 3).
            orientations1 (ndarray): Trajectory for platform 1 orientations (steps x 3).
            positions2 (ndarray): Trajectory for platform 2 positions (steps x 3).
            orientations2 (ndarray): Trajectory for platform 2 orientations (steps x 3).
        """
        # Generate trajectory for platform 1
        positions1, orientations1 = self.platform1.generate_trajectory(steps, dt)

        # Generate trajectory for platform 2 with a phase shift
        shift_steps = int(phase_shift * steps)
        positions2 = np.roll(positions1, shift_steps, axis=0)
        orientations2 = np.roll(orientations1, shift_steps, axis=0)

        return positions1, orientations1, positions2, orientations2

    def simulate(self, positions1, orientations1, positions2, orientations2, steps):
        """
        Simulates the motion of both platforms simultaneously.

        Parameters:
            positions1 (ndarray): Desired positions for platform 1 (steps x 3).
            orientations1 (ndarray): Desired orientations for platform 1 (steps x 3).
            positions2 (ndarray): Desired positions for platform 2 (steps x 3).
            orientations2 (ndarray): Desired orientations for platform 2 (steps x 3).
            steps (int): Number of simulation steps.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for step in range(steps):
            # Update platform 1
            position1 = positions1[step]
            orientation1 = orientations1[step]
            desired_lengths1 = self.platform1.inverse_kinematics(position1, orientation1)
            control_signals1 = self.platform1.pid_control(desired_lengths1)

            # Update platform 2
            position2 = positions2[step]
            orientation2 = orientations2[step]
            desired_lengths2 = self.platform2.inverse_kinematics(position2, orientation2)
            control_signals2 = self.platform2.pid_control(desired_lengths2)

            # Handle actuator movement for platform 1
            if self.platform1.serial_port:
                self.platform1.send_control_signal(control_signals1)
                encoder_feedback1 = self.platform1.read_encoder_feedback()
                if encoder_feedback1 is not None:
                    self.platform1.current_lengths = encoder_feedback1
            else:
                self.platform1.current_lengths += control_signals1 * self.dt

            # Handle actuator movement for platform 2
            if self.platform2.serial_port:
                self.platform2.send_control_signal(control_signals2)
                encoder_feedback2 = self.platform2.read_encoder_feedback()
                if encoder_feedback2 is not None:
                    self.platform2.current_lengths = encoder_feedback2
            else:
                self.platform2.current_lengths += control_signals2 * self.dt

            # Visualize both platforms
            if step % 5 == 0:
                ax.clear()
                self.platform1.visualize_platform(position1, orientation1, ax)
                self.platform2.visualize_platform(position2, orientation2, ax)
                plt.pause(0.01)

        plt.show()
