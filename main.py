from drone_sim.drone import Drone
from visualization.animator import Animator
import numpy as np

drone_params = {"mass": 2, "armLength": 0.265, "Ixx": 0.234, "Iyy": 0.234, "Izz": 0.468}
init_states = [0,  0,  -3,       # x,y,z  
               0,  0,  0,       # vx,vy,vz
               0,  0,  0,       # phi,theta,psi
               0,  0,  0]       # p,q,r    
init_inputs = [0, 0, 0, 0]     # Initial inputs u[0] = Torque, u[1:] = Moments
dt = 0.05

sim_time = 10 

Daedalus = Drone(params=drone_params, initInputs=init_inputs,initStates=init_states,dt=dt)

for _ in np.arange(0,sim_time, dt):
     Daedalus.update()

animator = Animator(Daedalus)
animator.show()
