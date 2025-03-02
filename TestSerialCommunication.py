import numpy as np
import serial
import time


def test_serial_communication(serial_port='COM5', baudrate=250000, num_actuators=6, test_values=None):
    """
    Test function to verify serial communication between Python and Arduino.
    Sends actuator lengths and receives feedback from the Arduino.
    """
    # Establish serial connection with Arduino
    arduino = serial.Serial(port=serial_port, baudrate=baudrate, timeout=1)
    time.sleep(5)  # Allow serial communication to establish

    # Default test values if none provided
    if test_values is None:
        test_values = np.linspace(45, 50, num_actuators)  # Example test actuator lengths

    try:
        # Send actuator lengths to Arduino
        send_string = ','.join(f'{value:.2f}' for value in test_values) + '\n'
        print("Sending to Arduino:", send_string.strip())
        arduino.write(send_string.encode())
        time.sleep(0.05)

        # Read feedback from Arduino
        feedback = arduino.readline().decode().strip()
        if feedback:
            try:
                feedback_values = np.array([float(val) for val in feedback.split(',')])
                print("Received from Arduino:", feedback_values)
            except ValueError:
                print("Warning: Invalid feedback received", feedback)
        else:
            print("No response received from Arduino.")

    except Exception as e:
        print(f"Error in communication: {e}")
    finally:
        arduino.close()
        print("Serial connection closed.")


# Run test
#test_serial_communication()


# Initialize Serial communication with Arduino
ser = serial.Serial('COM7', 250000)
time.sleep(3)
print("Comunicación iniciada")
while True:

    integers2send = [255, 0, 255, 0, 255, 0]
    data2send = ','.join(map(str, integers2send)) + '\n'
    ser.write(data2send.encode('utf-8'))
    time.sleep(0.5)
    data_received = ser.readline()
    lengths = data_received.decode('utf-8', errors='ignore')
    #print("Data received: ", lengths)
    time.sleep(0.5)

    if lengths[0] == 'A':
        lengths = lengths[1:].replace('\r\n', '').split(',')
        lengths = [float(x) for x in lengths]
        print(type(lengths), "Control received")
        print(lengths[1:])
    time.sleep(0.2)
