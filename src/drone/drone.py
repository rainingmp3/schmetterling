import numpy as np
from drone.b2w import RotationMatrices
from drone.dynamics import Dynamics

class Drone:
    '''
    
    Drone class  handles state evolution over simulation and logging. 

    '''
    def __init__(self, 
                 params: dict[str, float],
                 initStates: list, 
                 initInputs:list, 
                 gravity: float = 9.81,
                 dt: float  = 0.05):
        """
        Initialize the Drone object.

        Args:
            params : Drone physical parameters.
            initStates : Initial state vector.
            initInputs : Initial control input vector.
            gravity : Gravity constant. Defaults to 9.81 m/s^2.
            dt : Time step. Defaults to 0.05 s.
            
            """
        # DRONE MODEL
        self.params = params.copy()
        self.mass = self.params['mass']             # DRONE MASS [kg]   
        self.attached_mass = self.params['mass']    # ATTACHMENT MASS [kg]   
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
        self.dynamics = Dynamics(self)
        
        # TABLES
        self.time_table = []
        self.states_table = []
        self.rotation_matrix_table = []
        self.thrust_table = []
    
    # DYNAMICAL VARIABLES
    @property 
    def rotation_matrix(self) -> np.ndarray:
        """
        Get the rotation matrix from body to inertial frame.
        """
        return self.rotation.get_rotation_matrix()
    
    @property 
    def transformation_matrix(self) -> np.ndarray:
        """
        Get the transformation matrix for angular velocities 
        from body to inertial frame.
        """
        return self.rotation.get_transformation_matrix()
    
    def update_control(self, controlInput) -> None:
        """
        Update control input.

        Args:
            controlInput : Control signal vector.
        """
        self.u = controlInput 
        # UPDATING DEPENDED STATES
        self.T = self.u[0]
        self.M = controlInput[1:]
    
    def log_states(self) -> None:
        """
        Log current states for plotting and animation.
        """
        self.time_table.append(self.t)
        self.states_table.append(np.copy(self.x))
        self.rotation_matrix_table.append(np.copy(self.rotation_matrix))
        self.thrust_table.append(np.copy(self.T))   

    def update_states(self) -> None:
        """
        Update current states based on equation of motions.
        """
        dx = self.dynamics.EOM(self.x)
        self.x += dx * self.dt
        self.t += self.dt 
        
        # UPDATING DEPENDED STATES
        self.r = self.x[:3]                               
        self.dr = self.x[3:6]                             
        self.eule = np.radians(self.x[6:9])               
        self.w = np.radians(self.x[9:12])                 
   
    def update(self) -> None:
        """
        Update and log states.
        """
        self.update_states()
        self.log_states()
        # print(f"{self.states_table[-1][3]} on {self.t} s AND INPUT IS {self.u}")
