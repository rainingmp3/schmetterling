from drone_sim.drone import Drone
from visualization.animator import Animator
from drone_sim.controller import Controller
import numpy as np

drone_params = {"mass": 2, "armLength": 0.265, "Ixx": 0.234, "Iyy": 0.234, "Izz": 0.468}
init_states = [0,  0, 0,       # x,y,z  
               0,  0,  0,       # vx,vy,vz
               0,  0,  0,       # phi,theta,psi
               0,  0,  0]       # p,q,r    
init_inputs = [2 * 9.81,  0, 0, 0]     # Initial inputs u[0] = Torque, u[1:] = Moments

desired_states = [0,  0, -5,       # x,y,z  
                  0,  0,  0,       # vx,vy,vz
                  0,  0,  0,       # phi,theta,psi
                  0,  0,  0]       # p,q,r    

dt = 0.05

sim_time = 20

Daedalus = Drone(params=drone_params, initInputs=init_inputs,initStates=init_states,dt=dt)
controller = Controller(Daedalus, kp = 10 ,ki=0.0,kd=0.2,dt=dt)

for _ in np.arange(0,sim_time, dt):
     control_input = controller.compute_control(states=(Daedalus.x), desiredStates= desired_states)
     Daedalus.update_control(control_input)
     Daedalus.update()
     # print("shit")

animator = Animator(Daedalus, controller)
animator.show()
# print(Daedalus.rotation_matrix_table)