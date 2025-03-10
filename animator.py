import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rotation_matrix import RotationMatrix

class Animator:
    """3D animation in matplotlib"""
    def __init__(self, parent):
        self.parent = parent
        self.position = self.parent.x_table    # x,y,z
        self.length = self.parent.l
        self.dt = self.parent.dt
            
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection= '3d')
        self.ax.set_xlim(-10,10)   
        self.ax.set_ylim(-10,10)   
        self.ax.set_zlim(-10,10)   
        self.point, = self.ax.plot([], [], [], 'ro', label='Drone')
        self.arm1, = self.ax.plot([], [], [], color='cyan', linewidth=2)
        self.arm2, = self.ax.plot([], [], [], color='blue', linewidth=2)
        self.arm3, = self.ax.plot([], [], [], color='red', linewidth=2)  # New arm
        self.arm4, = self.ax.plot([], [], [], color='magenta', linewidth=2)  # New arm

        self.anime = FuncAnimation(self.fig,
                                self.animate,
                                np.arange(1,len(self.parent.time_table)),
                                interval = 2000,
                                blit = False,
                                init_func = self.init_anime)
    def init_anime(self):
        self.point.set_data([],[])
        self.arm1.set_data([],[])
        self.arm2.set_data([],[])
        self.arm3.set_data([],[])
        self.arm4.set_data([],[])
        self.point.set_3d_properties([])
        return self.point

    def animate(self, i):
        x = self.position[i][0]
        y = self.position[i][1]
        z = self.position[i][2]

        R = self.parent.rotation_matrix
        length = self.length


        arm1_end = R @ np.array([length, 0, 0])
        arm2_end = R @ np.array([0, length, 0])

        self.arm1.set_data([x-arm1_end[0], x + arm1_end[0]], [y - arm1_end[1], y + arm1_end[1]])
        self.arm1.set_3d_properties([z- arm1_end[2], z + arm1_end[2]])

        self.arm2.set_data([x - arm2_end[0], x + arm2_end[0]], [y - arm2_end[1], y + arm2_end[1]])
        self.arm2.set_3d_properties([z - arm2_end[2], z + arm2_end[2]])

        self.point.set_data([x], [y])
        self.point.set_3d_properties([z])
        return self.arm1, self.arm2, self.arm3, self.arm4, self.point
    
    def show(self):
            plt.show()