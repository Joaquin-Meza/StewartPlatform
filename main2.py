import time
import numpy as np
import matplotlib.pyplot as plt


def simulate_dual_platforms(platform1, platform2, positions1, orientations1, positions2, orientations2, steps, dt=0.1, arduino=None, update_interval=5, ideal_simulation=0):
    """
        Function to simulate the trajectories of Two Stewart Platforms side by side (Gait simulator) with seria communication for real-time control
    :param platform1: (StewartPlatform) The first Stewart platform instance.
    :param platform2: (StewartPlatform) The second Stewart platform instance.
    :param positions1: (ndarray) Trajectory positions for platform 1 (steps x 3)
    :param orientations1: (ndarray) Trajectory orientations for platform 1 (steps x 3).
    :param positions2: (ndarray) Trajectory positions for platform 2 (steps x 3).
    :param orientations2: (ndarray)  Trajectory orientations for platform 2 (steps x 3).
    :param steps: Number of simulation steps.
    :param dt: Time step between each simulation step (default: 0.1).
    :param update_interval: Interval for updating the visualization (default: 5).
    :param microcontroller:
    :return:
    """
    log = {
        'time_1': [],
        'desired_lengths_1': [],
        'current_lengths_1': [],
        'control_signals_1': [],
        'time_2': [],
        'desired_lengths_2': [],
        'current_lengths_2': [],
        'control_signals_2': []
    }

    # Set up the figure and 3D axes for visualization
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Command arduino to go home configuration
    # Retrieve the current position of the actuators

    # Initialize actuator lengths at home position
    platform1.current_lengths = platform1.inverse_kinematics(platform1.home_position, platform1.home_orientation)
    platform2.current_lengths = platform2.inverse_kinematics(platform2.home_position, platform2.home_orientation)
    try:
        emergency_stop_triggered = False    # Initialize emergency stop flag

        # Simulation loop
        for step in range(steps):
            # Get the current position and orientation for both platforms
            position1 = positions1[step]
            orientation1 = orientations1[step]

            position2 = positions2[step]
            orientation2 = orientations2[step]

            # Compute the desired actuator lengths for both platforms
            desired_lengths1 = platform1.inverse_kinematics(position1, orientation1)
            desired_lengths2 = platform2.inverse_kinematics(position2, orientation2)

            # Compute the control signals for both platforms
            errors_1 = desired_lengths1-platform1.current_lengths
            errors_2 = desired_lengths2-platform2.current_lengths

            control_signals1 = np.zeros(6)
            control_signals2 = np.zeros(6)
            for i in range(6):
                if platform1.controller_type == 'PID' and platform2.controller_type == 'PID':
                    control_signals1[i] = platform1.pid_control(desired_lengths1[i], i)
                    control_signals2[i] = platform2.pid_control(desired_lengths2[i], i)
                if platform1.controller_type =='DSTA' and platform2.controller_type == 'DSTA':
                    control_signals1[i] = platform1.dsta_controllers[i].derivative(errors_1[i])
                    control_signals2[i] = platform2.dsta_controllers[i].derivative(errors_2[i])

            control_signals1 = np.clip(control_signals1, -255, 255)
            control_signals2 = np.clip(control_signals2, -255, 255)

            # Handle actuator movement with ot without serial communication
            if ideal_simulation == 0:
                try:
                    # Send control signals to Arduino
                    control_string = ','.join(f'{int(signal+255)}' for signal in np.concatenate((control_signals1, control_signals2))) + '\n'
                    arduino.write(control_string.encode('utf-8'))
                    time.sleep(0.05)
                    try:
                        arduino.setTimeout(1)   # Set 1 second timeout for response
                        # Read actuator feedback from Arduino
                        feedback = arduino.readline().decode('utf-8', errors='ignore').strip()      # Read the incoming data, decode the 'utf-8' format and remove leading and trailing whitespaces
                        if feedback:
                            try:
                                feedback_values = np.array([float(val) for val in feedback.split(',')])
                                platform1.current_lengths = feedback_values[:6]
                                platform2.current_lengths = feedback_values[6:]
                            except ValueError:
                                print("Warning: Invalid feedback received", feedback)
                        else:
                            print("No feedback received from Arduino")
                    except Exception as e:
                        print(f"Serial communication error: {e}")
                except Exception as e:
                    print(f"Serial communication error: {e}")
            else:
                # Apply control signals to actuators
                # Update the current actuator lengths (emulating actuator responses)
                platform1.current_lengths += control_signals1 * dt
                platform1.current_lengths = np.clip(platform1.current_lengths, platform1.min_length, platform1.max_length)

                platform2.current_lengths += control_signals2 * dt
                platform2.current_lengths = np.clip(platform2.current_lengths, platform2.min_length, platform2.max_length)

            # Log the results
            log['time_1'].append(step*platform1.dt)
            log['desired_lengths_1'].append(desired_lengths1.copy())
            log['current_lengths_1'].append(platform1.current_lengths.copy())
            log['control_signals_1'].append(control_signals1.copy())
            log['time_2'].append(step * platform2.dt)
            log['desired_lengths_2'].append(desired_lengths2.copy())
            log['current_lengths_2'].append(platform2.current_lengths.copy())
            log['control_signals_2'].append(control_signals2.copy())


            # Update the visualization every `update_interval` steps
            if step % update_interval == 0:
                ax.clear()
                platform1.visualize_platform2(position1, orientation1, ax)
                platform2.visualize_platform2(position2, orientation2, ax)

                # Add titles and labels
                ax.set_title('Dual Stewart Platform Simulation')
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')

                plt.pause(0.1)
            plt.show()
            if emergency_stop_triggered:
                print("Emergency Stop Triggered! Stopping actuator...")
                if arduino:
                    arduino.write(b"EMERGENCY_STOP\n")
                break
    except KeyboardInterrupt:
        print("\nEmergency stop triggered! Stopping actuators...")
        emergency_stop_triggered = True
        if arduino:
            arduino.write(b"EMERGENCY_STOP\n")  # Send Stop command to arduino
        time.sleep(1)




def test_Actuator(platform, actuator, controller, steps, dt=0.1, arduino=None, update_interval=5):
    """
        Function to test actuator with real-time feedback visualization

        Parameters:
        platform: StewartPlatform instance controlling the actuator
        actuator: Actuator index(0-5)
        controller: 'PID' or 'DSTA' for control method
        steps: Number of steps in the test
        dt: Time step interval (default: 0.1s)
        arduino: Serial communication object
        update_interval: Frequency for updating the plot
    """
    log = {
        'time': [],
        'desired_lengths': [],
        'current_lengths': [],
        'control': []
    }

    fig, ax = plt.subplots(2, 1, figsize=(10, 8))

    # Define sinusoidal test trajectory
    T = 5  # Duration of test (seconds)
    t = np.linspace(0, T, steps)
    trajectory = 5 * np.cos(2 * np.pi * t / T)  # Cosine wave motion

    time.sleep(0.5)
    try:
        emergency_stop_triggered = False    # Initialize emergency stop flag
        # Simulation loop
        for step in range(steps):

            # Get the desired actuator position from trajectory
            position = trajectory[step]

            # Compute the error
            error = position - platform.current_lengths[actuator]

            control = 0

            if controller == 'PID':
                control = platform.pid_control(position, actuator)
            elif controller == 'DSTA':
                control = platform.dsta_controllers[actuator].derivative(error)

            # Send control signal and receive feedback
            try:
                # Send control command in forma: "ACTUATOR_TEST, <platform_index>,<actuator_index>,<control_signal>"
                control = max(min(int(control+255), 510), 0)
                control_str = f"ACTUATOR_TEST,{platform.platform_id},{actuator},{int(control)}\n"
                arduino.write(control_str.encode('utf-8'))
                time.sleep(0.05)

                # Read actuator feedback from Arduino
                feedback = arduino.readline().decode('utf-8', errors='ignore').strip()
                if feedback:
                    platform.current_lengths[actuator] = float(feedback)
            except ValueError:
                print("Warning: Invalid feedback received", feedback)

            # Log data for plotting
            log['time'].append(step * dt)
            log['desired_lengths'].append(position)
            log['current_lengths'].append(platform.current_lengths[actuator])
            log['control'].append(control)

            time_steps = log['time']
            desired_lengths = log['desired_lengths']
            current_lengths = log['current_lengths']
            control_trajectory = log['control']

            # Update the visualization every 'update_interval' steps
            if step % update_interval == 0:
                ax[0].clear()
                ax[1].clear()

                ax[0].plot(time_steps, desired_lengths, label='Desired lengths', linestyle='--')
                ax[0].plot(time_steps, current_lengths, label='Current lengths')
                ax[0].set_title('Desired vs Current Actuator Length')
                ax[0].set_xlabel('Time (s)')
                ax[0].set_ylabel('Position (cm)')
                ax[0].legend()
                ax[0].grid(True)

                ax[1].plot(time_steps, control_trajectory, label='Control Signal')
                ax[1].set_title('Control signal')
                ax[1].set_xlabel('Time (s)')
                ax[1].set_ylabel('Position (cm)')
                ax[1].legend()
                ax[1].grid(True)

                plt.tight_layout()
                plt.pause(0.1)
            plt.show()
            if emergency_stop_triggered:
                print("Emergency Stop Triggered! Stopping actuator...")
                if arduino:
                    arduino.write(b"EMERGENCY_STOP\n")
                break   # Exit main loop
    except KeyboardInterrupt:
        print("\nEmergency stop triggered! Stopping actuators...")
        emergency_stop_triggered = True
        if arduino:
            arduino.write(b"EMERGENCY_STOP\n")      # Send Stop command to arduino
        time.sleep(1)