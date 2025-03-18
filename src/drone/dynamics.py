import numpy as np

class Dynamics:
    """

    Computes equations of motion
    
    """
    def __init__(self, drone):
        self.drone = drone
        self.update_matrices

    @property
    def update_matrices(self):
        g = self.drone.g
        mass = self.drone.mass
        I = self.drone.I
        sin_psi = np.sin(self.drone.eule[2])
        cos_psi = np.cos(self.drone.eule[2])
        self.A = np.array([ [0,0,0,1,0,0,         0,         0,0,0,0,0],   
                            [0,0,0,0,1,0,         0,         0,0,0,0,0],
                            [0,0,0,0,0,1,         0,         0,0,0,0,0],
                            [0,0,0,0,0,0,-g*sin_psi,-g*cos_psi,0,0,0,0],
                            [0,0,0,0,0,0, g*cos_psi,-g*sin_psi,0,0,0,0],
                            [0,0,0,0,0,0,         0,         0,0,0,0,0],
                            [0,0,0,0,0,0,         0,         0,0,1,0,0],
                            [0,0,0,0,0,0,         0,         0,0,0,1,0],
                            [0,0,0,0,0,0,         0,         0,0,0,0,1],
                            [0,0,0,0,0,0,         0,         0,0,0,0,0],
                            [0,0,0,0,0,0,         0,         0,0,0,0,0],
                            [0,0,0,0,0,0,         0,         0,0,0,0,0],
                            ])
    
        self.B= np.array([  [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [    +1/mass,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [          0,     1/I[0][0],             0,             0],
                            [          0,             0,     1/I[1][1],             0],
                            [          0,             0,             0,     1/I[2][2]],
                            ])
        
        self.C =  np.array([[1,0,0,0,0,0,0,0,0,0,0,0],
                            [0,1,0,0,0,0,0,0,0,0,0,0],
                            [0,0,1,0,0,0,1,0,0,0,0,0],
                            [0,0,0,0,0,0,0,1,0,0,0,0],
                            [0,0,0,0,0,0,0,0,1,0,0,0]])

        self.D =  np.eye(6,4)
        

    def EOM(self, states):
        self.update_matrices

        self.R = np.array(self.drone.rotation_matrix)
        self.R_inv = np.linalg.inv(self.R)
        self.Tr = np.array(self.drone.transformation_matrix)
        dx = self.A @ states + self.B @ self.drone.u
        
        # FORCES
        grav_force = self.R_inv @ [0,0, self.drone.mass * self.drone.g] # from inertial to body
        external_forces = grav_force
        
        #MOMENTS
        gyro_moment = - np.cross(self.drone.w, self.drone.I @ self.drone.w)#####!!!!!!!!!!!
        moments = gyro_moment

        dx[3:6] += external_forces/self.drone.mass
        dx[3:6] = self.R @ dx[3:6]
        dx[6:9] = self.Tr @ dx[6:9]
        # dx[9:12] += moments #!!!!!!!!!!!!!!! NOT WORKING
        print(f"dx: {dx}, Thrust: {self.drone.u[0]}, Grav: {grav_force}")

        # print(f"{dx[5]}")
        return dx