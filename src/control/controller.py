import numpy as np

class Controller():
    """
    
    PID Controller for Drone.
    Uses proportional, derivative, and integral terms to compute
    control input. 
    
    """
    def __init__(self, 
                 drone, 
                 kp: float, 
                 ki: float,
                 kd: float,
                 dt: float) -> None: 
        """
        Initialize the PID controller.

        Args:
            drone: Drone object. 
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            dt: Time step.

        """
        
        self.drone = drone
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.dt = dt

        # Initial total error and previous time step error:
        self.prev_err = 0 
        self.sum_err  = 0 
        
        # Store error and desired state over simulation:
        self.err_table = []
        self.x_des_table = []

    def compute_control(self, 
                        states: list[float], 
                        desiredStates: list[float]) -> np.ndarray:
        """
        Compute control input signal via PID.
        Args:
            states: Current state vector.
            desiredStates: Desired state vector.
        Returns:
            Control signal vector u.
        """

        x =  states[2]
        x_des = desiredStates[2]
        
        # Proportional, derivative and integral error calculations:
        err  =  (x_des - x)
        diff_err = (err - self.prev_err)/self.dt
        self.sum_err += err
        
        # Clamp integral error to prevent wind up:
        self.sum_err = np.clip(self.sum_err, -10, 10)
        
        # Update previous error value:
        self.prev_err = err

        p_input = self.kp * err
        i_input = self.ki * self.sum_err
        d_input = self.kd * diff_err

        # Wrong code which doesnt work for now:
        control_input =  self.drone.mass * self.drone.g - ( p_input + i_input + d_input)
        xx = np.zeros(4)
        xx[0] = control_input 
        # print(f"Error: {err}, Thrust: {control_input}, p_input: {p_input}, i_input: {i_input} ")
        
        # Update error and desired state tables:
        self.err_table.append(err)
        self.x_des_table.append(x_des)
        return xx
         