import numpy as np
import copy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from rotation_matrix import RotationMatrix
from matrices import Matrices

class Drone:
    def __init__(self, params: dict[str, float], initStates: list, initInputs:list, gravity= 9.81, dt = 0.05):
        
        # DRONE MODEL
        self.params = params.copy()
        self._mass = self.params['mass']            # DRONE MASS [kg]   
        self._attached_mass = self.params['mass']   # ATTACHMENT MASS [kg]   
        self.g = gravity                            # ACCELERATION OF GRAVITY [m/s^2]
        self.l = self.params['armLength']           # DRONE ARM LENGTH [m]
        self.dt = dt                                # TIME STEP [s]
        self.t = 0                                  # CURRENT TIME [s]   
        
        # Tables
        self.time_table = []
        self.x_table = []                    # STATUS-VECTOR TABLE
        
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
        self.M = self.u[1:4]                               # TORQUE [N*m]
        
        # COMPOSITION LOGIC
        self.rotation = RotationMatrix(self)
        self.matrices = Matrices(self)
    # DYNAMICAL VARIABLES
    # STATE MATRICES
    @property
    # A(12 x 12) 
    def A(self):
        return self.matrices.A
    
    @property
    # B(12X4)
    def B(self):
        return self.matrices.B
    
    @property
    # C(6X12)
    def C(self):
        return self.matrices.C
    
    @property
    # D(6X4)
    def D(self):
        return self.matrices.D
    

    def update(self):
        self.dx = self.A @ self.x + self.B @ self.u 
        self.y  = self.C @ self.x + self.D @ self.u
        self.t += self.dt 

        self.time_table.append(self.t)
        self.x_table.append(self.x)
        # INTEGRATION SOLVER # 4now just Euler
        # 4further update: x(k) = x(k-1) + dx(k-1) * dt
        self.x = self.x + self.dx * self.dt

        