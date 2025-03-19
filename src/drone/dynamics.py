import numpy as np

class Dynamics:
    """

    Dynamics class computes equations of motion.
    Represents physical plant of the drone.
    In calculations rotation matrices included.

    
    """
    def __init__(self, drone):
        self.drone = drone
        
        """
        Initialize dynamics.

        Args:
            drone: Drone object to control

        """

    def EOM(self, states: list[float]) -> list[float]:
        """
        EOM function computes equations of motion for drone
        Args:
            states: Drones states
        Returns:
            dx: States derivative dx
        """
        self.R = np.array(self.drone.rotation_matrix)
        self.R_inv = np.linalg.inv(self.R)
        self.Tr = np.array(self.drone.transformation_matrix)
        dx = self.A @ states + self.B @ self.drone.u
    
        # Forces acting on a drone computations:
        # Gravity force is computed in the inertial frame.
        # Rotate it into the body frame using the inverse rotation matrix.
        grav_force = self.R_inv @ [0,0, self.drone.mass * self.drone.g] # from inertial to body
        external_forces = grav_force
        
        # Moments acting on a drone computations:
        gyro_moment = - np.cross(self.drone.w, self.drone.I @ self.drone.w)#####!!!!!!!!!!!
        moments = gyro_moment
        # Apply forces and moments, then rotate from body to inertial 
        dx[3:6] += external_forces/self.drone.mass
        dx[3:6] = self.R @ dx[3:6]
        dx[6:9] = self.Tr @ dx[6:9]
        # dx[9:12] += moments #!!!!!!!!!!!!!    !! NOT WORKING
        print(f"dx: {dx}, Thrust: {self.drone.u[0]}, Grav: {grav_force}")

        # print(f"{dx[5]}")
        return dx