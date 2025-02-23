import numpy as np
import copy
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation

class Drone:
    def __init__(self, params: dict[str, float], initStates: list, initInputs:list, gravity= 9.81, dt = 0.05):
        
        # DRONE MODEL
        self.params = params.copy()
        self.mass = self.params['mass']      # DRONE MASS [kg]   
        self.g = gravity                     # ACCELERATION OF GRAVITY [m/s^2]
        self.l = self.params['armLength']    # DRONE ARM LENGTH [m]
        self.dt = dt
        
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
        
        # STATE MATRICES
        # A(12 x 12) 
        self.A = np.array([ [0,0,0,1,0,0,0,     0,      0,0,0,0],   
                            [0,0,0,0,1,0,0,     0,      0,0,0,0],
                            [0,0,0,0,0,1,0,     0,      0,0,0,0],
                            [0,0,0,0,0,0,0,     0,-self.g,0,0,0],
                            [0,0,0,0,0,0,0,self.g,      0,0,0,0],
                            [0,0,0,0,0,0,0,     0,      0,0,0,0],
                            [0,0,0,0,0,0,0,     0,      0,1,0,0],
                            [0,0,0,0,0,0,0,     0,      0,0,1,0],
                            [0,0,0,0,0,0,0,     0,      0,0,0,1],
                            [0,0,0,0,0,0,0,     0,      0,0,0,0],
                            [0,0,0,0,0,0,0,     0,      0,0,0,0],
                            [0,0,0,0,0,0,0,     0,      0,0,0,0],
                            ])
        # B(12X4)
        self.B = np.array([[          0,             0,             0,             0],
                           [          0,             0,             0,             0],
                           [          0,             0,             0,             0],
                           [          0,             0,             0,             0],
                           [1/self.mass,             0,             0,             0],
                           [          0,             0,             0,             0],
                           [          0,             0,             0,             0],
                           [          0,             0,             0,             0],
                           [          0,             0,             0,             0],
                           [          0,1/self.I[0][0],             0,             0],
                           [          0,             0,1/self.I[1][1],             0],
                           [          0,             0,             0,1/self.I[2][2]],
                           ])
        # C(6X12)
        self.C = np.array([[1,0,0,0,0,0,0,0,0,0,0,0],
                           [0,1,0,0,0,0,0,0,0,0,0,0],
                           [0,0,1,0,0,0,0,0,0,0,0,0],
                           [0,0,0,0,0,0,1,0,0,0,0,0],
                           [0,0,0,0,0,0,0,1,0,0,0,0],
                           [0,0,0,0,0,0,0,0,1,0,0,0]])
        # D(6X4)
        self.D = np.array([[0,0,0,0],
                           [0,0,0,0],
                           [0,0,0,0],
                           [0,0,0,0],
                           [0,0,0,0],
                           [0,0,0,0]])
    
    def update(self):
        self.dx = self.A @ self.x + self.B @ self.u 
        self.y  = self.C @ self.x + self.D @ self.u
        
        # INTEGRATION SOLVER # 4now just Euler
        # 4further update: x(k) = x(k-1) + dx(k-1) * dt
        self.x = self.x + self.dx * self.dt
        print(self.dx)
        pass

    def animating(self):
        
    