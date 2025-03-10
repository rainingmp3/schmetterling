import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Animator:
    """3D animation in matplotlib"""
    def __init__(self, drone):
        self.drone = drone
        self.position = self.drone.x_table    # x,y,z
        self.dt = self.drone.dt
            
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection= '3d')
        self.ax.set_xlim(-10,10)   
        self.ax.set_ylim(-10,10)   
        self.ax.set_zlim(-10,10)   
        self.point, = self.ax.plot([], [], [], 'ro', label='Drone')
        self.anime = FuncAnimation(self.fig,
                                self.animate,
                                np.arange(1,len(self.drone.time_table)),
                                interval = 2000,
                                blit = False,
                                init_func = self.init_anime)
    def init_anime(self):
        self.point.set_data([],[])
        self.point.set_3d_properties([])
        return self.point

    def animate(self,i):
        self.point.set_data([self.position[i][0]],
                            [self.position[i][1]])
        self.point.set_3d_properties([[self.position[i][2]]])
        print(f"{self.position[i][1]} and {self.position[i][2]}")
        return self.point
    
    def show(self):
            plt.show()