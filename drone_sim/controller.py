class Controller():
    '''Controller for Drone'''
    def __init__(self, kp, ki, kd, dt): 
        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.prev_err = 0 
        self.dt = dt

    def compute_control(self, states, desiredStates):
        x = states
        x_des = desiredStates
        
        return None
         