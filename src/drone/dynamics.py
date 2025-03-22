import numpy as np

class Dynamics:
    """

    Dynamics class computes equations of motion.
    Represents physical plant of the drone.
    In calculations rotation matrices included.

    
    """
    def __init__(self, drone):
        self.drone = drone
        self.dx = np.zeros(12)

        
        """
        Initialize dynamics.

        Args:
            drone: Drone object to control

        """

    def EOM(self) -> list[float]:
        """
        EOM function computes equations of motion for drone
        Returns:
            dx: States derivative dx
        """
        self.R = np.array(self.drone.rotation_matrix)
        self.Tr = np.array(self.drone.transformation_matrix)
        # Forces acting on a drone computations:
        # Gravity force is computed in the inertial frame.
        # Rotate it into the body frame using the tramsportation matrix
        # As R is orthonormal => R^(-1) = R.T
        grav_force = self.R.T @ [0,0, self.drone.mass * self.drone.g] # from inertial to body
        thrust_force =   [0,0,-self.drone.T]
        external_forces = grav_force + thrust_force
        
        # Moments acting on a drone computations:
        thrust_moment =np.linalg.inv(self.drone.I) @ (self.drone.M - np.cross(self.drone.w, self.drone.I @ self.drone.w))#####!!!!!!!!!!!
        moments = thrust_moment
        # Apply forces and moments, then rotate from body to inertial 
        self.dx[:3] = self.drone.dr
        self.dx[3:6] = external_forces/self.drone.mass
        self.dx[3:6] = self.R @ self.dx[3:6]
        self.dx[6:9] = self.Tr @ self.drone.w
        self.dx[9:12] = moments #!!!!!!!!!!!!!    !! NOT WORKING
        # print(f"dx: {self.dx}, Thrust: {self.drone.u[0]}, Grav: {grav_force}")

        # print(f"{dx[5]}")
        return self.dx
    
    def update_states(self) -> None:
        """
        Update current states based on equation of motions.
        """
        dx = self.EOM()
        self.drone.x += dx * self.drone.dt
        self.drone.t += self.drone.dt 
        
        # UPDATING DEPENDED STATES
        self.drone.r = self.drone.x[:3]                               
        self.drone.dr = self.drone.x[3:6]                             
        self.drone.eule = np.radians(self.drone.x[6:9])               
        self.drone.w = np.radians(self.drone.x[9:12])      