import numpy as np

class Controller():
    '''Controller for Drone'''
    def __init__(self, kp, ki, kd, dt): 
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt

        self.prev_err = 0 
        self.sum_err  = 0 

    def compute_control(self, states, desiredStates):
        x =  np.concatenate((np.array([states[2]]), np.array(states[6:9])))
        x_des = np.concatenate((np.array([desiredStates[2]]), np.array(desiredStates[6:9])))
        err = np.subtract(x,x_des)
        diff_err = np.subtract(x, self.prev_err)/self.dt

        p_input = self.kp * err
        i_input = self.ki * self.sum_err
        d_input = self.kd * diff_err

        self.sum_err += err
        self.prev_err = err
        print(err[0])
        return p_input + i_input + d_input
         