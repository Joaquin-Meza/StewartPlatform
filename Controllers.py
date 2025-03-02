import numpy as np

class STA:
    def __init__(self, tau, l1, l2, w1=0.0, w2=0.0):
        self.tau = tau
        self.l1 = l1
        self.l2 = l2
        self.w1 = [w1]
        self.w2 = [w2]

    def sign(self, value):
        #if value < 0:
        #    out = -1
        #else:
        #    out = 1
        return np.sign(value)

    def derivative(self, variable):
        """
        Implement the Super-Twisting Algorithm for sliding mode control
        :param variable:
        :return:
        """
        error = self.w1[-1] - variable      # Compute the error

        # Update controller states
        w1_aux = self.w1[-1] + self.tau * (self.w2[-1] - self.l1*(np.sqrt(np.abs(error))) * self.sign(error))
        w2_aux = self.w1[-1] + self.tau * (self.l2 * self.sign(error))

        # Store new values
        self.w1.append(w1_aux)
        self.w2.append(w2_aux)
        return w2_aux   # Return control signal



class DSTA(STA):
    """
    Class for Discrete Super Twisting Algorithm
    Discrete Super-Twisting Algorithm (DSTA) with additional damping factors rho1 and rho2
    """
    def __init__(self, tau, l1, l2, rho1=0.999, rho2=0.999, w1=0.01, w2=0.01):
        super().__init__(tau, l1, l2, w1, w2)
        self.rho1 = rho1
        self.rho2 = rho2

    def derivative(self, variable):
        """
        Implements the Discrete Super-Twisting Algorithm (DSTA) with damping.
        """
        error = self.w1[-1] - variable  # Compute error

        w1_aux = self.rho1 * self.w1[-1] + self.tau * (self.w2[-1] - self.l1 * (np.sqrt(np.abs(error))) * np.sign(error))
        w2_aux = self.rho2 * self.w2[-1] + self.tau * (-self.l2 * self.sign(error))

        # Store new values
        self.w1.append(w1_aux)
        self.w2.append(w2_aux)

        return w2_aux   # Return control signal


def plot_log_data(filename):
    import json
    import matplotlib.pyplot as plt
    # Load the log
    with open("simulation_log.json", "r") as file:
        log = json.load(file)

    # Plot errors over time
    time = log["time"]
    errors = np.array(log["errors"])
    control_signals = np.array(log["control_signals"])

    # Plot errors over time
    plt.figure(figsize=(12, 6))
    plt.subplot(2,1,1)
    for i in range(6):  # Plot each motor's error
        plt.plot(time, errors[:, i], label=f"Motor {i + 1}")

    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.title("Motor Errors Over Time")
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)
    for i in range(6):
        plt.plot(time, control_signals[:,i], label=f"Motor {i+1}")

    plt.xlabel("Time (s)")
    plt.ylabel("Control Signal")
    plt.title("Control Signals Over Time")
    plt.legend()
    plt.grid()
    plt.show()

    plt.tight_layout()
    plt.show()