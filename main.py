import control as ctrl  
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from simple_pid import PID
import pygad
from drone import Drone
from animator import Animator

Daedalus = Drone(
    params={"mass": 2, "armLength": 0.265, "Ixx": 0.234, "Iyy": 0.234, "Izz": 0.468},
    initStates=[0, 0, -1,       # x,y,z  
                0, 0, 0,        # vx,vy,vz
                0, 0, 0,        # phi,theta,psi
                0, 0, 0],       # p,q,r    
    initInputs=[10, 0, 0, 1])   # Initial inputs u[0] = Torque, u[1:] = Moments

for i in range(5):
    Daedalus.update()

animator = Animator(Daedalus)
animator.show()



