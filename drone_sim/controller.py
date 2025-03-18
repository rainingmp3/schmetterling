import numpy as np

class Controller():
    '''Controller for Drone'''
    def __init__(self, drone, kp, ki, kd, dt): 
        self.drone = drone
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt

        self.prev_err = 0 
        self.sum_err  = 0 
        
        self.err_table = []
        self.x_des_table = []

    def compute_control(self, states, desiredStates):
        x =  states[2]
        x_des = desiredStates[2]
        err  =  -(x_des - x)
        diff_err = (err - self.prev_err)/self.dt

        self.sum_err += err
        self.prev_err = err

        p_input = self.kp * err
        i_input = self.ki * self.sum_err
        d_input = self.kd * diff_err
        control_input =   self.drone.mass * self.drone.g + p_input + i_input + d_input
        xx = np.zeros(4)
        xx[0] = control_input 
        print(f"Error: {err}, Thrust: {control_input}")
        self.err_table.append(err)
        self.x_des_table.append(x_des)
        return xx
         