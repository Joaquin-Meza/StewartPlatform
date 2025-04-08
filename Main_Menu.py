import time
from main2 import simulate_dual_platforms, test_Actuator, go2setpoint
import serial
from StewartPlatform import StewartPlatform
from PlatformTrajectory import PlatformTrajectory
import json
import numpy as np


def connect_to_arduino(port=None, baudrate=250000):
    arduino = None  # Placeholder for the serial object

    if port is not None:    # Only initialize if a port is provided
        print("Setting serial communication...")
        arduino = serial.Serial(port, baudrate)
        time.sleep(3)   # Allow serial communication to establish
        print("Serial communication established")

    return arduino


def test_actuator(serial_object, platform_idx, actuator_idx, target_position):
    """
    Function to test specific actuator's performance
    :param serial_object: Serial object  t to communicate with the microcontroller and send commands
    :param platform_idx:  Indicates to which platform the actuator belongs
    :param actuator_idx:  Indicates which actuators will be tested
    :param target_position: Indicates the position to set the actuator
    :return:
    """
    print("Begin test function")
    # Ensure index is valid
    if platform_idx not in [0, 1]:
        print("Invalid platform index. Choose between 0 and 1.")
        return
    if actuator_idx < 0 or actuator_idx > 5:
        print("Invalid actuator index. Choose between and 5.")
        return

    # Send command to Arduino
    command = f"TEST_ACTUATOR,{platform_idx},{actuator_idx},{target_position}"
    serial_object.write(command.encode('utf-8'))
    time.sleep(0.1)

    # Read response
    while True:
        if serial_object.in_waiting > 0:
            response = serial_object.readline().decode('utf-8', errors='ignore').strip()
            print(f"Arduino Response: {response}")
            if "Test Complete" in response or "Test Failed" in response:
                break
    print("Test finished. ")


def menu(platform1, platform2, positions1, orientations1, positions2, orientations2, steps, dt=0.1, serial_port=None):
    """
    Main function to control the gait simulator, considering a menu interface to select the tasks to be carried out
    :param platform1:
    :param platform2:
    :param positions1:
    :param orientations1:
    :param positions2:
    :param orientations2:
    :param steps:
    :param dt:
    :param update_interval:
    :param serial_port:
    :param baudrate:
    :return:
    """

    # Establish serial connection with Microcontroller
    arduino = connect_to_arduino(serial_port)

    # Check if the connection is set correctly
    if arduino is None:
        print("No Arduino connection established.")
    else:

        # Define log structure to save data if required
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

        while True:
            print("\n Select the required action: ")
            print("1 - Move platform to home configuration")
            print("2 - Get current actuator positions")
            print("3 - Run experiment")
            print("4 - Save log files")
            print("5 - Test Actuators")
            print("6 - Go to set point")
            print("7 - Exit")

            choice = input("Which action will be carried out?")

            if choice == '1':   # Add a FUNCTION to command arduino to set platforms to home configuration
                arduino.flushInput()  # Ensure buffer is clean
                arduino.write(b'HOME\n')    # Send home command
                time.sleep(0.1)     # Allow arduino to process
                while arduino.in_waiting == 0:
                    time.sleep(0.1)     # Wait until response is available
                print("Moving to home position")

                response = arduino.readline().decode().strip()
                time.sleep(1)
                print("Arduino response: ", response)

            elif choice == '2':     # Add a FUNCTION to command arduino send the readings of the current position
                arduino.write(b'POSITION\n')    # Request actuator position
                print("Requiring actuator positions...")
                time.sleep(1)
                response = arduino.readline().decode().strip()
                print("Current actuator positions: ", response)

            elif choice == '3':
                print("Starting experiment...")
                time.sleep(1)
                #arduino.write(b'TEST\n')

                # Simulate the dual platforms
                simulate_dual_platforms(
                    platform1=platform1,
                    platform2=platform2,
                    positions1=positions1,
                    orientations1=orientations1,
                    positions2=positions2,
                    orientations2=orientations2,
                    steps=steps,
                    dt=dt,
                    update_interval=1,  # Update visualization every 5 steps
                    arduino=arduino,
                    ideal_simulation=0
                )

            elif choice == '4':
                with open("simulation_log.json", "w") as file:
                    json.dump(log, file, indent=4)
                print("Log file saved as simulation_log.json")

            elif choice == '5':  # Call actuator testing function
                try:
                    platform_idx = int(input("Enter platform index(0 or 1): "))
                    actuator_idx = int(input("Enter actuator index (0-5): "))
                    controller_type = input("Enter control type ('PID' or 'DSTA'): ").strip().upper()
                    steps_sim = int(input("Enter number of test steps: "))

                    # Valid input
                    if platform_idx not in [0, 1] or actuator_idx not in range(6):
                        print("Invalid platform or actuator index.")
                        continue
                    if controller_type not in ['PID', 'DSTA']:
                        print("Invalid controller type. Please enter 'PID' or 'DSTA'. ")
                        continue

                    # Select the correct object platform
                    selected_platform = platform1 if platform_idx == 0 else platform2

                    # Call the actuator test function
                    test_Actuator(selected_platform, actuator_idx, controller_type, steps_sim, dt=0.1, arduino=arduino)

                except ValueError:
                    print("Invalid input. Please enter numbers only.")
            elif choice == '6':
                print("Go to set point")
                position1 = np.array(eval(input("Enter the desired Position for platform 1 as [x, y, z]: ")))
                orientation1 = np.array(eval(input("Enter the desired Orientations for platform 1 as [roll, pitch, yaw]: ")))
                position2 = np.array(eval(input("Enter the desired Position for platform 2 as [x, y, z]: ")))
                orientation2 = np.array(eval(input("Enter the desired Orientations for platform 2 as [roll, pitch, yaw]: ")))
                go2setpoint(platform1, platform2,
                            positions1=position1, orientations1=orientation1,
                            positions2=position2, orientations2=orientation2,
                            arduino=arduino, dt=0.1, ideal_simulation=0)
            elif choice == '7':
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please try again.")

        arduino.close()






# Initialize trajectory generator
#trajectory_generator = PlatformTrajectory(v_apoyo=50, v_oscilacion=10, z_max=50, theta_max=15,y_max=10, roll_max=5, pitch_max=5, T=5, num_cycles=3)
trajectory_generator = PlatformTrajectory(v_apoyo=50, v_oscilacion=20, z_max=30, theta_max=0.5, y_max=10, roll_max=5, pitch_max=10, T=1, num_cycles=1)

# Simulation parameters
dt = 0.1  # Time step
expected_time = 10  # Total simulation time
steps = int(expected_time / dt)  # Total steps

# Offset the platforms for side-by-side placement
offset = 25  # Distance between the two platforms along the Y-axis
# Create object instances for two platforms

platform1 = StewartPlatform(0, r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, -offset, 0])     # Right platform
platform2 = StewartPlatform(1, r_B=25, r_P=15, gamma_B_deg=25.25, gamma_P_deg=21.85, serial_port=None, controller_type='DSTA', base_offset=[0, offset, 0])      # Left platform
#platform1.mirror_axis('x')
#platform2.mirror_axis('x')
# Generate synchronized trajectories
positions1, orientations1, positions2, orientations2 = trajectory_generator.generate_trajectory2(steps, dt, platform1, platform2)
#positions1, orientations1, positions2, orientations2 = trajectory_generator.generate_orientation_test_trajectory(steps, dt, platform1, platform2)
orientations2 = orientations1
positions1[:, 1] -= offset  # Left platform (negative Y direction)
positions2[:, 1] += offset  # Right platform (positive Y direction)

# Parameters
dt = 0.5
steps = len(positions1)  # Ensure steps match the trajectory length

# Begin main loop
menu(platform1, platform2, positions1, orientations1, positions2, orientations2, steps, dt=0.0001, serial_port='COM5')
