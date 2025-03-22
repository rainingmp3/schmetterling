import numpy as np
import matplotlib.pyplot as plt

class Plotter:
    """"
    Plotter class handles states and inputs plotting over simulation.
    """
    def __init__(self, drone, controller):
        self.drone = drone
        self.controller = controller
        """
        Initialize Plotter class.
        Args:
            drone: drone object whose states are plotted
            controller: controller object whose control inputs are plotted
        """

    def plot_stats(self):
        """
        def plot_stats plots statistics of the whole simulation.
        Call it in main.py

        """
        fig,axs = plt.subplots(5, 1, figsize=(10,6),sharex=True)
        time =  np.array(self.drone.time_table)
        states = np.array(self.drone.states_table)
        desired = np.array(self.controller.x_des_table)
        lables = ["X","Y","Z", "dX", "dY","dZ",
                   "phi", "theta", "psi", "p", "q", "r"]
        titles = ["Position [m]", "Velocity [m/s]", 
                  "Attitude [rad]", "Angular velocity [rad/s]"]
        ylabels = ["X,Y,Z", "dX, dY,dZ", 
                   "phi, theta, psi", "p, q, r"]
        colors = ["red", "green","blue"]
        # Drones states plotting:
        for i in range(4):
            for j in range(1,4):
                axs[i].plot(time, states[:,i*3 + (j-1)], 
                            label = lables[i*3 + (j-1)],
                            color = colors[j-1])
                axs[i].set_title(titles[i])
                axs[i].set_xlabel("Time [s]")
                axs[i].set_ylabel(ylabels[i])
                axs[i].grid()
                axs[i].set_xlim(min(time), max(time))
                axs[i].legend()

        axs[0].plot(time, desired, label='Desired', color = 'magenta')

        # Thrust and control input plotting:
        axs[4].plot(time, self.drone.thrust_table, label='Thrust', color = 'red')
        axs[4].plot(time, self.controller.err_table, label='Error_z [m]', color = "blue")
        axs[4].set_title('Input thrust [m]')
        axs[4].set_xlabel('Time [s]')
        axs[4].set_ylabel('Input [N]')
        axs[4].set_xlim(min(time), max(time))
        axs[4].grid()
        axs[4].legend()

        plt.tight_layout()
        plt.show()

    