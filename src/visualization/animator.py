import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Animator:
    """
    Animator class plays 3D animation of drone over simulation.
    
    """
    def __init__(self, drone):
        """
        Initialize Animator class.
        Args:
            drone: drone object to animate 
        """
        # Define drone object to draw
        self.drone = drone
        self.length = self.drone.l
        self.dt = self.drone.dt
        
    def init_anime(self):
        """
        init_anime is inner function for matplotlibs FuncAnimation
        class.
        Initializes drone body containers where later drones 
        position and attitude are going to be stored for drawing.

        Returns:
            Line objets for FuncAnimation.
        """
        self.point.set_data([],[])
        self.arm1.set_data([],[])
        self.arm2.set_data([],[])
        self.point.set_3d_properties([])
        self.arm1.set_3d_properties([])
        self.arm2.set_3d_properties([])
        return self.point, self.arm1, self.arm2

    def animate(self, i):
        """
        init_anime is inner function for matplotlibs FuncAnimation
        class.
        Populates created by init_anime containers with positional data
        for drawing.

        Returns:
            Updated objects for FuncAnimation.
        """
        x = self.drone.states_table[i][0]
        y = self.drone.states_table[i][1]
        z = self.drone.states_table[i][2]
        R = self.drone.rotation_matrix_table[i]
        t = self.drone.time_table[i]

        # Lock animation on drone while it flies:
        self.ax.set_xlim(x - 5, x + 5)   
        self.ax.set_ylim(y - 5, y + 5)   
        self.ax.set_zlim(z + 5, z - 5)   

        # Uses rotation matrix to rotate drones body in world frame. 
        length = self.length
        arm1_end = R @ np.array([length, 0, 0])
        arm2_end = R @ np.array([0, length, 0])

        self.arm1.set_data([x-arm1_end[0], x + arm1_end[0]], [y - arm1_end[1], y + arm1_end[1]])
        self.arm1.set_3d_properties([z- arm1_end[2], z + arm1_end[2]])

        self.arm2.set_data([x - arm2_end[0], x + arm2_end[0]], [y - arm2_end[1], y + arm2_end[1]])
        self.arm2.set_3d_properties([z - arm2_end[2], z + arm2_end[2]])

        self.point.set_data([x], [y])
        self.point.set_3d_properties([z])

        self.ax.legend([self.point],[f"Time:{t:.2f} s"])
        
        return self.arm1, self.arm2, self.point
    
    def play(self):
        """
        def play runs the animation of the drone.
        Call it in main.py
        """
        self.fig = plt.figure(figsize=(6, 4))   
        self.ax = self.fig.add_subplot(111, projection= '3d')
        
        self.ax.set_xlabel("X[m]")
        self.ax.set_ylabel("Y[m]")
        self.ax.set_zlabel("Z[m]")
        self.point, = self.ax.plot([], [], [], 'ro', label='Drone')
        self.arm1, = self.ax.plot([], [], [], color='cyan', linewidth=2)
        self.arm2, = self.ax.plot([], [], [], color='blue', linewidth=2)

        self.anime = FuncAnimation(self.fig,
                        self.animate,
                        frames=len(self.drone.states_table),
                        interval=self.drone.t/len(self.drone.states_table) * 1000,
                        blit = False,
                        init_func = self.init_anime,
                        repeat=True)
        plt.show()
    
