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
    log_1 = {
        'time': [],
        'desired_lengths': [],
        'current_lengths': [],
        'control_signals': []
    }
    log_2 = {
        'time': [],
        'desired_lengths': [],
        'current_lengths': [],
        'control_signals': []
    }


    # Set up the figure and 3D axes for visualization
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    if arduino != None:
        # Command arduino to go home configuration
        arduino.flushInput()  # Ensure buffer is clean
        #arduino.write(b'HOME\n')  # Send home command
        time.sleep(0.500)  # Allow arduino to process (seconds)

        # Retrieve the current position of the actuators
        arduino.write(b'POSITION\n')  # Request actuator position
        print("Requiring actuator positions...")
        time.sleep(1)
        feedback = arduino.readline().decode('utf-8', errors='ignore').strip()
        if feedback:
            try:
                feedback_values = np.array([float(val) for val in feedback.split(',')])
                platform1.current_lengths = feedback_values[:6]
                platform2.current_lengths = feedback_values[6:]
            except ValueError:
                print("Warning: Invalid feedback received", feedback)


    # Initialize actuator lengths at home position
    platform1.current_lengths = platform1.inverse_kinematics(platform1.home_position, platform1.home_orientation)
    platform2.current_lengths = platform2.inverse_kinematics(platform2.home_position, platform2.home_orientation)
    try:
        emergency_stop_triggered = False    # Initialize emergency stop flag
        arduino.flushInput()

        # Simulation loop
        for step in range(steps):
            # Get the current desired position and orientation for both platforms
            position1 = positions1[step]
            orientation1 = orientations1[step]

            position2 = positions2[step]
            orientation2 = orientations2[step]

            # Compute the desired actuator lengths for both platforms using IK
            desired_lengths1 = platform1.inverse_kinematics(position1, orientation1)
            desired_lengths2 = platform2.inverse_kinematics(position2, orientation2)
            print(f"Desired lengths: P1: {desired_lengths1} P2: {desired_lengths2}")
            print(f"Current lengths: P1: {platform1.current_lengths}, P2:{platform2.current_lengths}")
            # Compute the error for both platforms actuators
            errors_1 = desired_lengths1-platform1.current_lengths
            errors_2 = desired_lengths2-platform2.current_lengths

            # Compute the control signals for both platforms
            control_signals1 = np.zeros(6)
            control_signals2 = np.zeros(6)
            for i in range(6):
                if platform1.controller_type == 'PID' and platform2.controller_type == 'PID':
                    control_signals1[i] = platform1.pid_control(desired_lengths1[i], i)
                    control_signals2[i] = platform2.pid_control(desired_lengths2[i], i)
                if platform1.controller_type =='DSTA' and platform2.controller_type == 'DSTA':
                    control_signals1[i] = platform1.dsta_controllers[i].derivative(errors_1[i])
                    control_signals2[i] = platform2.dsta_controllers[i].derivative(errors_2[i])
            print(f"Actuators CONTROL: P1[{control_signals1}], P2 [{control_signals2}]")
            control_signals1 = [round(control_bounds(x), 0) for x in control_signals1]
            control_signals2 = [round(control_bounds(x), 0) for x in control_signals2]

            # Handle actuator movement with or without serial communication
            if ideal_simulation == 0:
                try:
                    # Send control signals to Arduino
                    control_string = "TEST," +','.join(f'{int(signal)}' for signal in np.concatenate((control_signals1, control_signals2))) + '\n'
                    arduino.write(control_string.encode('utf-8'))

                    print(f"Sending command: {control_string}")     # Debug line - Show what is being sent
                    time.sleep(1)
                    try:
                        # Debug check if Arduino has data available
                        # Time out for serial read to prevent indefinite waiting
                        start_time = time.time()
                        while arduino.in_waiting == 0:
                            if time.time() - start_time > 5:    # Wait after 2 seconds
                                print("Timeout waiting for Arduino response")
                                break
                        # Read actuator feedback from Arduino
                        feedback = arduino.readline().decode('utf-8', errors='ignore').strip()      # Read the incoming data, decode the 'utf-8' format and remove leading and trailing whitespaces
                        print(f"Feedback received: {feedback}")
                        if feedback == "":
                            continue        # Skip empty lines
                        if feedback:
                            try:
                                feedback_values = np.array([float(val) for val in feedback.split(',')])
                                platform1.current_lengths = feedback_values[:6]
                                platform2.current_lengths = feedback_values[6:]
                                print(f"Actuators current length: P1[{platform1.current_lengths}], P2 [{platform2.current_lengths}]")
                            except ValueError:
                                print("Warning: Invalid feedback received", feedback)
                        else:
                            print("No feedback received from Arduino")
                    except Exception as e:
                        print(f"Serial communication error: {e}")
                except Exception as e:
                    print(f"Serial communication error: {e}")
            else:
                print("Simulation without real time feedback")
                # Apply control signals to actuators
                # Update the current actuator lengths (emulating actuator responses)
                platform1.current_lengths += (control_signals1 * dt)
                platform1.current_lengths = np.clip(platform1.current_lengths, platform1.min_length, platform1.max_length)

                platform2.current_lengths += (control_signals2 * dt)
                platform2.current_lengths = np.clip(platform2.current_lengths, platform2.min_length, platform2.max_length)

            # Log the results
            log_1['time'].append(step*platform1.dt)
            log_1['desired_lengths'].append(desired_lengths1.copy())
            log_1['current_lengths'].append(platform1.current_lengths.copy())
            log_1['control_signals'].append(control_signals1.copy())
            log_2['time'].append(step * platform2.dt)
            log_2['desired_lengths'].append(desired_lengths2.copy())
            log_2['current_lengths'].append(platform2.current_lengths.copy())
            log_2['control_signals'].append(control_signals2.copy())


            # Update the visualization every `update_interval` steps
            """
            if step % update_interval == 0:
                ax.clear()
                platform1.visualize_platform2(position1, orientation1, ax)
                platform2.visualize_platform2(position2, orientation2, ax)

                # Add titles and labels
                ax.set_title('Dual Stewart Platform Simulation')
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')

            plt.pause(0.1)"""

            # Wait for user confirmation in terminal
            arduino.write(b'STOP\n')
            input("Press Enter to proceed to the next movement...")

            if emergency_stop_triggered:
                print("Emergency Stop Triggered! Stopping actuator...")
                if arduino:
                    arduino.write(b"EMERGENCY_STOP\n")
                break
        plt.show()
        # Stop actuators movement - Send control signals to 0
        arduino.write(b'STOP\n')
        # Plot platform performance
        platform1.plot_actuator_response(log_1)
        platform2.plot_actuator_response(log_2)
        #platform1.plot_actuator_rmse()
        #platform2.plot_actuator_rmse()
    except KeyboardInterrupt:
        print("\nEmergency stop triggered! Stopping actuators...")
        emergency_stop_triggered = True
        if arduino:
            arduino.write(b"EMERGENCY_STOP\n")  # Send Stop command to arduino
        time.sleep(1)

def control_bounds(x):
    if x >= 255:
        x = 255
    elif x <= -255:
        x = -255
    return x


def test_Actuator(platform, actuator, controller, steps, dt=0.1, arduino=None, update_interval=5):
    """
        Function to test actuator with real-time feedback visualization

        Parameters:
        platform: StewartPlatform instance controlling the linear actuator's
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
    trajectory = 2 * np.cos(2 * np.pi * t / T)+5  # Cosine wave motion

    time.sleep(0.5)
    try:
        emergency_stop_triggered = False    # Initialize emergency stop flag
        arduino.flushInput()                # Clear any old data in the serial buffer

        # Simulation loop
        for step in range(steps):

            # Get the desired actuator position from trajectory
            position = trajectory[step]


            # Compute the error
            error = position - platform.current_lengths[actuator]
            print(f"Error: {error}, Current position: {platform.current_lengths[actuator]}")

            control = 0

            if controller == 'PID':
                control = platform.pid_control(position, actuator)
            elif controller == 'DSTA':
                control = platform.dsta_controllers[actuator].derivative(error)
                print("Control", control)

            # Send control signal and receive feedback
            try:
                # Send control command in forma: "ACTUATOR_TEST, <platform_index>,<actuator_index>,<control_signal>"
                #control = round(control_bounds(control), 0)
                control_str = f"ACTUATOR_TEST,{platform.platform_id},{actuator},{control}\n"


                print(f"Sending command: {control_str}")    # Debug: Show what is sent
                arduino.write(control_str.encode('utf-8'))
                time.sleep(0.05)

                # Debug check if Arduino has data Available
                # Time out for serial read to revent indefinite waiting
                start_time = time.time()
                while arduino.in_waiting == 0:
                    if time.time() - start_time > 2:    # Time after 2 second
                        print("Timeout Waiting for Arduino response")
                        break
                # Read actuator feedback from Arduino
                feedback = arduino.readline().decode('utf-8', errors='ignore').strip()
                print(f"Feedback received: {feedback}")
                if feedback == "":
                    continue        # Skip empty lines
                if feedback:     # Ensure feedback is a valid number   .replace(".","",).isdigit()
                    platform.current_lengths[actuator] = float(feedback)
                    print(f"Actuator current length: {platform.current_lengths[actuator]}")
                else:
                    print(f"Invalid feedback: {feedback} ")
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
                ax[1].set_ylabel('Gain')
                ax[1].legend()
                ax[1].grid(True)

                plt.tight_layout()
                plt.pause(0.1)

            if emergency_stop_triggered:
                print("Emergency Stop Triggered! Stopping actuator...")
                if arduino:
                    arduino.write(b"EMERGENCY_STOP\n")
                break   # Exit main loop
        plt.show()
        arduino.write(b"STOP\n")
    except KeyboardInterrupt:
        print("\nEmergency stop triggered! Stopping actuators...")
        emergency_stop_triggered = True
        if arduino:
            arduino.write(b"EMERGENCY_STOP\n")      # Send Stop command to arduino
        time.sleep(1)