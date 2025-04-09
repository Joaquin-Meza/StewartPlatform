import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.interpolate import CubicSpline
import pandas as pd

class PlatformTrajectory:
    def __init__(self, v_apoyo=100, v_oscilacion=50, z_max=70, min_length=45, theta_max=15, T=1, num_cycles=1, y_max=10, roll_max=5, pitch_max=5):
        """
        Initializes the trajectory generator with parameters.

        Parameters:
            v_apoyo (float): Linear velocity during support phase (mm/s).
            v_oscilacion (float): Linear velocity during swing phase (mm/s).
            z_max (float): Maximum height during swing phase (mm).
            theta_max (float): Maximum yaw rotation (degrees).
            y_max (float): Maximum lateral movement (mm).
            roll_max (float): Maximum roll rotation (degrees).
            pitch_max (float): Maximum pitch rotation (degrees).
            T (float): Duration of one gait cycle (s).
            num_cycles (int): Number of gait cycles to generate.
        """
        self.v_apoyo = v_apoyo
        self.v_oscilacion = v_oscilacion
        self.z_max = z_max
        self.min_length = min_length
        self.theta_max = theta_max
        self.T = T
        self.num_cycles = num_cycles
        self.y_max = y_max
        self.roll_max = roll_max
        self.pitch_max = pitch_max

    def scale_signal(self, y, target_min, target_max):
        y_min = np.min(y)
        y_max = np.max(y)
        if y_max == y_min:
            return np.full_like(y, target_min)  # Avoid dived by zero
        A = target_max-target_min
        return A*(y-y_min)/(y_max-y_min)+target_min

    def generate_trajectory4(self, steps, dt, stewart_platform1, stewart_platform2):
        """
        Generates a trajectory for the platform simulating a walking cycle in place using both platforms.

        Parameters:
            steps (int): Number of steps in the trajectory.
            dt (float): Time interval between steps.

        Returns:
            positions1 (ndarray): Desired positions for platform 1 over time (steps x 3).
            orientations1 (ndarray): Desired orientations for platform 1 over time (steps x 3).
            positions2 (ndarray): Desired positions for platform 2 over time (steps x 3).
            orientations2 (ndarray): Desired orientations for platform 2 over time (steps x 3).
        """
        # Time vector for the trajectory
        t = np.linspace(0, self.T * self.num_cycles, steps)

        # Apply a smoothing function to avoid abrupt transitions between cycles
        smooth_factor = 0.5 * (1 - np.cos(2 * np.pi * t / self.T))  # Smooth start and end of each cycle

        # Compute min and max actuator lengths for each platform
        home_lengths1 = stewart_platform1.inverse_kinematics(stewart_platform1.home_position,
                                                             stewart_platform1.home_orientation)
        home_lengths2 = stewart_platform2.inverse_kinematics(stewart_platform2.home_position,
                                                             stewart_platform2.home_orientation)

        min_length1 = np.min(home_lengths1) + 5
        max_length1 = min_length1 + 20

        min_length2 = np.min(home_lengths2) + 5
        max_length2 = min_length2 + 20


        # Define the stepping pattern  - Change X and Y positions
        Y = 20 * np.sin(2 * np.pi * t / self.T) * smooth_factor  # Forward-backward sinusoidal motion in X (-40 to 40 cm)
        Z = self.min_length + (self.z_max-self.min_length)*(1-np.cos(2*np.pi*t/self.T))/2 -40 # Vertical oscillation in Z
        X = np.zeros_like(Y)  # No movement in Y


        # Orientation (pitch only)
        # Restrict pitch angle to avoid complete rotation
        Pitch = self.theta_max * np.sin(2 * np.pi * t / self.T) * smooth_factor  # Smooth sinusoidal yaw rotation
        Pitch = np.clip(Pitch, -30, 30)
        # Ensure the pitch movement remains continuous and does not flip due to discontinuities
        Pitch = np.unwrap(Pitch*np.pi/180)*180/np.pi    # Prevent abrupt transitions in pitch
        Pitch = np.zeros_like(Pitch)
        Roll = np.zeros_like(Pitch)  # No roll
        Yaw = np.zeros_like(Pitch)  # No Yaw
        # Generate the second platform trajectory with a half-phase shift
        Y2 = np.roll(X, steps // 2)
        Z2 = np.roll(Z, steps // 2)

        X2 = np.zeros_like(Y2)
        Pitch2 = np.roll(Pitch, steps // 2)

        # Combine positions and orientations for both platforms
        positions1 = np.vstack([X, Y, Z]).T  # Positions: [X, Y, Z] for platform 1
        orientations1 = np.vstack([Roll, Pitch, Yaw]).T  # Orientations: [roll, pitch, yaw] for platform 1
        positions2 = np.vstack([X2, Y2, Z2]).T  # Positions: [X, Y, Z] for platform 2
        orientations2 = np.vstack([Roll, Pitch2, Yaw]).T  # Orientations: [roll, pitch, yaw] for platform 2

        return positions1, orientations1, positions2, orientations2

    def generate_trajectory2(self, steps, dt, stewart_platform1, stewart_platform2):
        # Time vector for the trajectory
        t = np.linspace(0, self.T * self.num_cycles, steps)

        # Trajectory parameters
        stride_length = 10          # Total travel in Y (cm)
        base_z1 = stewart_platform1.home_position[2]
        base_z2 = stewart_platform2.home_position[2]
        safe_margin = 2  # Margin from physical limits (in cm)

        # Smooth sinusoidal stepping pattern
        # Platform 1: moves forward and lift
        Y1 = (stride_length/2)*np.sin(1*np.pi*t/self.T)

        # Lift profile (0 to 1)
        raw_lift = np.sin(np.pi*t/self.T)**2

        # Define actuator range for each platform
        min_z1 = stewart_platform1.min_length + safe_margin
        max_z1 = stewart_platform1.max_length - safe_margin
        min_z2 = stewart_platform2.min_length + safe_margin
        max_z2 = stewart_platform2.max_length - safe_margin

        # Apply scaling to raw lift profile
        Z1 = self.scale_signal(raw_lift, 10, 15)
        Z2 = self.scale_signal(np.roll(raw_lift, steps//2), 10, 15)

        # Phase shift Y for platform 2
        Y2 = np.roll(Y1, steps//2)

        # No lateral motion in X
        X1 = np.zeros_like(Y1)
        X2 = np.zeros_like(Y2)

        #No rotation
        Roll1 = Pitch1 = Yaw1 = np.zeros_like(t)
        Roll2 = Pitch2 = Yaw2 = np.zeros_like(t)

        # Combine into position and orientation arrays
        positions1 = np.vstack([X1, Y1, Z1]).T
        orientations1 = np.vstack([Roll1, Pitch1, Yaw1]).T
        positions2 = np.vstack([X2, Y2, Z2]).T
        orientations2 = np.vstack([Roll2, Pitch2, Yaw2]).T

        return positions1, orientations1, positions2, orientations2



    def generate_orientation_test_trajectory(self, steps, dt, stewart_platform1, stewart_platform2):
        """
        Generates a trajectory where only orientation changes to test the Stewart Platform's rotational behavior.

        Parameters:
            steps (int): Number of steps in the trajectory.
            dt (float): Time interval between steps.

        Returns:
            positions (ndarray): Fixed position for the platform (steps x 3).
            orientations (ndarray): Rotational movements (steps x 3).
        """
        # Time vector
        t = np.linspace(0, self.T * self.num_cycles, steps)

        # Damped oscillation envelope (simulate wobble decay)
        damping = np.exp(-1.5 * t / (self.T * self.num_cycles))  # Higher value = faster decay

        # Oscillate roll and pitch with 90° phase difference (like wobbling coin)
        theta_max = self.theta_max  # Max angle in degrees (keep small, e.g., 2°–5°)

        Roll = theta_max * np.sin(2 * np.pi * t / self.T) * damping
        Pitch = theta_max * np.cos(2 * np.pi * t / self.T) * damping
        Yaw = np.zeros_like(Roll)  # No yaw motion

        # Unwrap angles to avoid discontinuities (optional for small angles)
        Roll = np.unwrap(Roll * np.pi / 180) * 180 / np.pi
        Pitch = np.unwrap(Pitch * np.pi / 180) * 180 / np.pi

        # No translation
        X = np.zeros_like(Roll)
        Y = np.zeros_like(Roll)
        base_z = stewart_platform1.home_position[2]
        Z = 5 * (np.sin(2 * np.pi * t / self.T))**2

        base_z2 = stewart_platform2.home_position[2]
        Z2 = 5 * (np.sin(2 * np.pi * t / self.T))**2

        # Second platform: out of phase (180° shift)
        Roll2 = np.roll(Roll, steps // 2)
        Pitch2 = np.roll(Pitch, steps // 2)
        Yaw2 = np.zeros_like(Roll2)

        positions1 = np.vstack([X, Y, Z]).T
        orientations1 = np.vstack([Roll, Pitch, Yaw]).T
        positions2 = np.vstack([X, Y, Z2]).T
        orientations2 = np.vstack([Roll2, Pitch2, Yaw2]).T

        return positions1, orientations1, positions2, orientations2

    def generate_wave_trajectory(self, steps):
        """
        Generates a wave-like trajectory to simulate undulating motion.

        Parameters:
            steps (int): Number of steps in the trajectory.
            dt (float): Time interval between steps.

        Returns:
            positions (ndarray): Wave-like positions over time (steps x 3).
            orientations (ndarray): Wave-like orientations over time (steps x 3).
        """
        # Time vector
        t = np.linspace(0, self.T * self.num_cycles, steps)

        # Define wave-like motion
        X = 20 * np.sin(2 * np.pi * t / self.T)  # Sinusoidal motion in X
        Y = 20 * np.cos(2 * np.pi * t / self.T)  # Circular motion in Y
        Z = self.min_length + 5 + 3 * np.sin(4 * np.pi * t / self.T)  # Vertical wave motion in Z

        # Define wave-like rotations
        Roll = 10 * np.sin(3 * np.pi * t / self.T)  # Roll oscillation
        Pitch = 15 * np.sin(2 * np.pi * t / self.T)  # Pitch oscillation
        Yaw = 10 * np.sin(3 * np.pi * t / self.T)  # Yaw oscillation

        positions = np.vstack([X, Y, Z]).T
        orientations = np.vstack([Roll, Pitch, Yaw]).T

        return positions, orientations

    def trajectory3(self,steps, dt, sw1,sw2):
        # Time vector for the trajectory
        t = np.linspace(0, self.T*self.num_cycles, steps)

        # Apply smoothing function
        smooth_factor = 0.5 * (1 - np.cos(2*np.pi*t / self.T))

        # Compute min and max actuator lengths for each platform
        home_lengths1 = sw1.inverse_kinematics(sw1.home_position, sw1.home_orientation)
        home_lengths2 = sw2.inverse_kinematics(sw2.home_position, sw2.home_orientation)

        min_length1 = np.min(home_lengths1) + 5
        max_length1 = min_length1 + 20
        min_length2 = np.min(home_lengths2) + 5
        max_length2 = min_length2 + 20

        # Define the stepping pattern with an arched trajectory
        X = 40 * np.sin(2 * np.pi * t / self.T) * smooth_factor  # Forward-backward sinusoidal motion in X (-400 to 400 mm)
        Z = self.min_length + (self.z_max - self.min_length) * (1 - np.cos(2 * np.pi * t / self.T)) / 2  # Arch-shaped motion in Z
        Y = np.zeros_like(X)  # No movement in Y

        # Compute ΔX and ΔZ for pitch calculation
        delta_X = np.diff(X, prepend=X[0])  # Approximate derivative for X
        delta_Z = np.diff(Z, prepend=Z[0])  # Approximate derivative for Z


        # Compute pitch angle using atan2
        Pitch = np.degrees(np.arctan2(delta_X, delta_Z))  # Convert to degrees
        Pitch = gaussian_filter1d(Pitch, sigma=5)

        #Pitch = 10 * np.sin(2 * np.pi*t/self.T)
        # Clip pitch to avoid extreme angles
        Pitch = np.clip(Pitch, -15, 15)  # Limit pitch between -20 and 15 degrees

        Roll = np.zeros_like(Pitch)  # No roll
        Yaw = np.zeros_like(Pitch)  # No yaw

        # Generate the second platform trajectory with a half-phase shift
        X2 = np.roll(X, steps // 2)
        Z2 = np.roll(Z, steps // 2)
        Y2 = np.zeros_like(X2)

        # Compute pitch for the second platform
        delta_X2 = np.diff(X2, prepend=X2[0])
        delta_Z2 = np.diff(Z2, prepend=Z2[0])


        Pitch2 = np.degrees(np.arctan2(delta_X2, delta_Z2))
        Pitch2 = gaussian_filter1d(Pitch2, sigma=5)
        #Pitch2 = 10 * np.sin(2 * np.pi *t/self.T+np.pi)
        Pitch2 = np.clip(Pitch2, -15, 15)

        # Combine positions and orientations for both platforms
        positions1 = np.vstack([X, Y, Z]).T  # Positions: [X, Y, Z] for platform 1
        orientations1 = np.vstack([Roll, Pitch, Yaw]).T  # Orientations: [roll, pitch, yaw] for platform 1
        positions2 = np.vstack([X2, Y2, Z2]).T  # Positions: [X, Y, Z] for platform 2
        orientations2 = np.vstack([Roll, Pitch2, Yaw]).T  # Orientations: [roll, pitch, yaw] for platform 2

        return positions1, orientations1, positions2, orientations2


    def generate_trajectory_from_data(self, steps, sw1, sw2):
        # Load cleaned dataset
        df = pd.read_csv('cleaned_trajectory.csv')
        time = df["Time"].values
        ankle_angle = df["Ankle_Angle"].values

        # Normalize time values to fit the number of steps
        time = np.linspace(0, max(time), steps)

        # Smooth the ankle angle values to avoid abrupt changes
        ankle_angle_smooth = gaussian_filter1d(np.interp(time, df["Time"], ankle_angle), sigma=5)

        # Define X, Y, Z positions based on time
        X = 40 * np.sin(2 * np.pi * time / max(time))  # Sinusoidal motion in X
        Z = 50 + 10 * np.cos(2 * np.pi * time / max(time))  # Arch-like motion in Z
        Y = np.zeros_like(X)  # No lateral movement

        # Assign ankle angle to platform Pitch
        Pitch = np.clip(ankle_angle_smooth, -15, 15)

        Roll = np.zeros_like(Pitch)  # No roll for simplicity
        Yaw = np.zeros_like(Pitch)  # No yaw for simplicity

        # Generate second platform trajectory (half-phase shift)
        X2 = np.roll(X, steps // 2)
        Z2 = np.roll(Z, steps // 2)
        Y2 = np.zeros_like(X2)

        # Compute pitch for second platform
        Pitch2 = np.roll(Pitch, steps // 2)

        # Combine positions and orientations for both platforms
        positions1 = np.vstack([X, Y, Z]).T
        orientations1 = np.vstack([Roll, Pitch, Yaw]).T
        positions2 = np.vstack([X2, Y2, Z2]).T
        orientations2 = np.vstack([Roll, Pitch2, Yaw]).T

        return positions1, orientations1, positions2, orientations2

    def generate_oscillatory_trajectory(self, steps):
        """
        Generates an oscillatory motion for the Stewart platform without translational displacement.

        Parameters:
            steps (int): Number of steps in the trajectory.
            dt (float): Time interval between steps.

        Returns:
            positions (ndarray): Fixed positions over time (steps x 3).
            orientations (ndarray): Oscillatory orientations over time (steps x 3).
        """

        # Time vector
        t = np.linspace(0, self.T * self.num_cycles, steps)

        # No translation: The platform remains in place
        X = np.zeros(steps)  # Fixed X position
        Y = np.zeros(steps)  # Fixed Y position
        Z = np.full(steps, self.min_length + 5)  # Keep Z fixed at a reference height

        # Define oscillatory motion in orientation
        Roll = 10 * np.sin(2 * np.pi * t / self.T)  # Roll oscillation
        Pitch = 15 * np.sin(2 * np.pi * t / self.T)  # Pitch oscillation
        Yaw = 10 * np.sin(3 * np.pi * t / self.T)  # Yaw oscillation

        # Combine into arrays
        positions = np.vstack([X, Y, Z]).T  # Fixed positions
        orientations = np.vstack([Roll, Pitch, Yaw]).T  # Oscillatory orientations

        return positions, orientations