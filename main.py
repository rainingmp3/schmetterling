"""
Main script to initilize and run the drone simulation.
- Sets up initial conditions and drone parameters
- Runs the simulation loop to compute control input 
  and update states via Drone and Controller classes
- Visualizes simulation via Animator class

The drone has "+" shape configuration.
Off-diagonal terms of matrix of inertia are assumed to be 0.

Author: Kotanov Ivane
No rights reserved though :)
"""

from src.drone.drone import Drone
from src.control.controller import Controller
from src.visualization.animator import Animator
from src.visualization.plotter import Plotter
import numpy as np

# Drone physical characteristics:
drone_params = {
     "mass": 1.25,
     "armLength": 0.256,
     "Ixx": 0.232, 
     "Iyy": 0.232,
     "Izz": 0.0468
     }

# Initial positional states of the drone in inertial frame: 
# [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
init_states = [
               0,  0, 0,       # Position (x,y,z)  
               0,  0,  0,      # Velocity (vx,vy,vz) 
               0,  0,  0,      # Attitude (phi,theta,psi)
               0,  0,  0       # Angular Velocity (p,q,r)    
               ]

# Initial control inputs:
# [thrust, moment_x, moment_y, moment_z]
init_inputs = [0, 0, 0, 0]

# Desired states vector:
# [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
desired_states = [
                  0,  0, -5,       # Position (x,y,z)  
                  0,  0,  0,       # Velocity (vx,vy,vz)
                  0,  0,  0,       # Attitude (phi,theta,psi)
                  0,  0,  0        # Angular Velocity (p,q,r)    
                  ]
                  

# Timestep and simulation time setting
dt = 0.05
sim_time = 50

# Create Drone and Controller objects
schmetterling = Drone(params=drone_params,
                 initInputs=init_inputs,
                 initStates=init_states,
                 dt=dt)
controller = Controller(schmetterling, kp = 1.1 ,ki=0.00,kd=0.0,dt=dt)


# Run simulation loop
for _ in np.arange(0,sim_time, dt):
     # Compute control input based on difference between desired state and current
     control_input = controller.compute_control(states=(schmetterling.x), 
                                                desiredStates= desired_states)
     schmetterling.update()
     schmetterling.update_control(control_input)

# Visualize simulation results
animator = Animator(schmetterling)
plotter = Plotter(schmetterling, controller)

# animator.play()
plotter.plot_stats()
