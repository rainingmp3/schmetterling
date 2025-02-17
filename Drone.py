import numpy as np
import copy

class Drone:
    def __init__(self, params: dict[str, float], initStates: list, initInputs:list, gravity= 9.81):
        
        # DRONE MODEL
        self.params = params.copy()
        self.mass = self.params['mass']      # DRONE MASS [kg]   
        self.g = gravity                     # ACCELERATION OF GRAVITY [m/s^2]
        self.l = self.params['armLength']    # DRONE ARM LENGTH [m]
        
        #INERTIA MATRIX [kg*m^2] 
        self.I    = np.array([[self.params['Ixx'],            0,             0],      
                              [0            ,self.params['Iyy'],             0],
                              [0            ,            0,self.params['Izz']]])
        # STATE VARIABLES
        self.x = np.array(initStates, dtype=float).copy()                 # STATE VECTOR 
        self.r = self.x[:3]                 # POSITION VECTOR [m]
        self.dr = self.x[3:6]               # VELOCITY VECTOR [m/s]
        self.eule = np.radians(self.x[6:9]) # EULER ANGLES [rad]
        self.w = np.radians(self.x[9:12])   # ANGULAR VELOCITIES [rad/s]
        
        # CONTROL INPUTS
        self.u = np.array(initInputs, dtype=float).copy()           # CONTROL INPUT 
        self.T = self.u[0]                                          # THRUST [N]
        self.M = self.u[1:4]                                        # TORQUE [N*m]
        
    
    def apply_forces(self):
        print("forces applied, thrust me!")
        pass
    