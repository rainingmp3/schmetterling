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
        fig,axs = plt.subplots(2, 1, figsize=(5,5),sharex=True)
        time =  np.array(self.drone.time_table)
        states = np.array(self.drone.states_table)
        
        # Drones states plotting:
    #  axs[0].plot(time, states[:,0], label='x', color='red')
    #  axs[0].plot(time, states[:,1], label='y', color='green')
        axs[0].plot(time, states[:,2], label='z', color='blue')
        axs[0].plot(time, self.controller.x_des_table, label='Desired', color='red')
        print(states[0,2])
        axs[0].set_title('Position [m]')
        axs[0].set_xlabel('Time [s]')
        axs[0].set_ylabel('X,Y,Z')
        axs[0].grid()
        axs[0].set_xlim(min(time), max(time))
        axs[0].legend()

        # Thrust and control input plotting:
        axs[1].plot(time, self.drone.thrust_table, label='Thrust')
        axs[1].plot(time, self.controller.err_table, label='Error_z [m]')
        axs[1].set_title('Input thrust [m]')
        axs[1].set_xlabel('Time [s]')
        axs[1].set_ylabel('Input [N]')
        axs[1].set_xlim(min(time), max(time))
        axs[1].grid()
        axs[1].legend()

        plt.tight_layout()
        plt.show()

    