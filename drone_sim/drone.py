import numpy as np
from drone_sim.rotation_matrix import RotationMatrices
from utils.state_matrices import StateMatrices

class Drone:
    '''Stores and updates states and parameters.'''
    def __init__(self, params: dict[str, float], initStates: list, initInputs:list, gravity= 9.81, dt = 0.05):
        
        # DRONE MODEL
        self.params = params.copy()
        self.mass = self.params['mass']            # DRONE MASS [kg]   
        self.attached_mass = self.params['mass']   # ATTACHMENT MASS [kg]   
        self.g = gravity                            # ACCELERATION OF GRAVITY [m/s^2]
        self.l = self.params['armLength']           # DRONE ARM LENGTH [m]
        self.dt = dt                                # TIME STEP [s]
        self.t = 0                                  # CURRENT TIME [s]   
        
        # INERTIA MATRIX [kg*m^2] 
        self.I    = np.array([[self.params['Ixx'],                 0,                  0],      
                              [0                 ,self.params['Iyy'],                  0],
                              [0                 ,                 0, self.params['Izz']]])
        # STATE VARIABLES
        self.x = np.array(initStates, dtype=float).copy()  # STATE VECTOR 
        self.r = self.x[:3]                                # POSITION VECTOR [m]
        self.dr = self.x[3:6]                              # VELOCITY VECTOR [m/s]
        self.eule = np.radians(self.x[6:9])                # EULER ANGLES [rad]
        self.w = np.radians(self.x[9:12])                  # ANGULAR VELOCITIES [rad/s]
        
        # CONTROL INPUTS
        self.u = np.array(initInputs, dtype=float).copy()  # CONTROL INPUT 
        self.T = self.u[0]                                 # THRUST [N]
        self.M = self.u[1:]                                # TORQUE [N*m]
        
        # COMPOSITION LOGIC
        self.rotation = RotationMatrices(self.eule)
        self.dynamics = StateMatrices(self)
        
        # TABLES
        self.time_table = [self.t]
        self.states_table = [self.x]                           # STATUS-VECTOR TABLE
        self.rotation_matrix_table = [self.rotation_matrix]
    
    # DYNAMICAL VARIABLES
    @property 
    def rotation_matrix(self):
        return self.rotation.get_rotation_matrix()
    
    @property 
    def transformation_matrix(self):
        return self.rotation.get_transformation_matrix()
    
    def log_states(self):
        self.time_table.append(self.t)
        self.states_table.append(np.copy(self.x))
        self.rotation_matrix_table.append(np.copy(self.rotation_matrix))

    def update_states(self):
        self.t += self.dt 
        self.dx = self.dynamics.eom(self.x,self.u)
        self.x += self.dx * self.dt
   
    def update(self):
        self.update_states()
        self.log_states()
        print(f"{self.states_table[-1][:3]} on {self.t} s")
