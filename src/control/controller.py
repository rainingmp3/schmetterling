import numpy as np
from simple_pid import PID
class Controller():
    """
    
    PID Controller for Drone.
    Uses proportional, derivative, and integral terms to compute
    control input. 
    
    """
    def __init__(self, 
                 drone, 
                 gains,
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
        self.gains = gains
        self.dt = dt
        self.control_input = np.zeros(4)

        # Initial total error and previous time step error:
        self.prev_err = 0 
        self.sum_err  = 0 
        
        # Store error and desired state over simulation:
        self.err_table = []
        self.x_des_table = []
    def compute_control(self,desiredStates):
        self.attitude_control(desiredStates)
    def attitude_control(self, 
                        desiredStates: list[float]) -> np.ndarray:
        """
        Compute control input signal via PID.
        Args:
            states: Current state vector.
            desiredStates: Desired state vector.
        Returns:
            Control signal vector u.
        """

        self.x_des = np.concatenate(([desiredStates[5]], desiredStates[6:9]))
        self.x = np.concatenate(([self.drone.x[5]], self.drone.eule))
        
        # Proportional, derivative and integral error calculations:
        self.err = (self.x_des - self.x)

        diff_err = (self.err - self.prev_err)/self.dt
        self.sum_err += self.err
        
        # Clamp integral error to prevent wind up:
        self.sum_err = np.clip(self.sum_err, -20, 20)
        
        # Update previous error value:
        self.prev_err = self.err


        self.drone.u[1] = (self.gains["kP_phi"]* self.err[1] + 
                     self.gains["kI_phi"]* self.err[1] + 
                     self.gains["kD_phi"]* (0 - self.drone.w[1])) 
        
        self.drone.u[2] = (self.gains["kP_theta"]* self.err[2] + 
                   self.gains["kI_theta"]* self.err[2] + 
                   self.gains["kD_theta"]* (self.err[3] - self.prev_err[3]))
        
        self.drone.u[3] = (self.gains["kP_psi"]* self.err[3] + 
                   self.gains["kI_psi"]* self.err[3] + 
                   self.gains["kD_psi"]* (self.err[3] - self.prev_err[3]))
        
            
        self.drone.u[0] = self.drone.mass * self.drone.g - (self.gains["kP_zdot"]* self.err[0] + 
                   self.gains["kI_zdot"]* self.err[0] + 
                   self.gains["kD_zdot"]* (self.err[0] - self.prev_err[0])) 
        
        self.prev_err = self.err
        self.sum_err += self.err
               
        # Update error and desired state tables:
        self.err_table.append(self.err)
        self.x_des_table.append(self.x_des)
        self.update_control()
        print(f"Error: {self.err}, Gravity: {self.drone.mass * self.drone.g},Thrust: {self.drone.u[0]}")
     
    def update_control(self) -> None:
        """
        Update control input.

        """
        # UPDATING DEPENDED STATES
        self.drone.T = self.drone.u[0]
        self.drone.M = self.drone.u[1:]