from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from math import *

fig = plt.figure()
ax = fig.add_subplot(111) #, projection='3d'

#setting
ax.set_xlim(-5,5)
ax.set_ylim(-5,5)
#ax.set_zlim(-5,5)
ax.set_xlabel('x')
ax.set_ylabel('y')
#ax.set_zlabel('z')
ax.grid()

f1, = ax.plot([], [], "r-", lw=1) #plot1

def gen():
    for phi in np.linspace(0,2*pi,100):
        yield np.cos(phi), np.sin(phi), phi

def update(data):
    p1, q1, psi = data
    f1.set_data(p1,q1)
    #f1.set_3d_properties(psi)

ani = animation.FuncAnimation(fig, update, gen, blit=False, interval=100, repeat=True)
ani.save('matplot003.gif', writer='imagemagick')
plt.show()