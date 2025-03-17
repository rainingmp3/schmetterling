import numpy as np

class StateMatrices:
    """Stores state matrcies ABCD"""
    def __init__(self, drone):
        self.drone = drone
        self.update_matrices()

    def update_matrices(self):
        g = self.drone.g
        mass = self.drone.params['mass']
        I = self.drone.I
        self.A = np.array([ [0,0,0,1,0,0,0,     0,      0,0,0,0],   
                            [0,0,0,0,1,0,0,     0,      0,0,0,0],
                            [0,0,0,0,0,1,0,     0,      0,0,0,0],
                            [0,0,0,0,0,0,0,    -g,      0,0,0,0],
                            [0,0,0,0,0,0,g,     0,      0,0,0,0],
                            [0,0,0,0,0,0,0,     0,      0,0,0,0],
                            [0,0,0,0,0,0,0,     0,      0,1,0,0],
                            [0,0,0,0,0,0,0,     0,      0,0,1,0],
                            [0,0,0,0,0,0,0,     0,      0,0,0,1],
                            [0,0,0,0,0,0,0,     0,      0,0,0,0],
                            [0,0,0,0,0,0,0,     0,      0,0,0,0],
                            [0,0,0,0,0,0,0,     0,      0,0,0,0],
                            ])
    
        self.B= np.array([  [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [          0,             0,             0,             0],
                            [     1/mass,             0,             0,             0],
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
          

    def eom(self, states):
        dx = self.A @ states + self.B @ self.drone.u
        dx[3:6] += 1/self.drone.mass * np.array(([0,0,self.drone.mass * self.drone.g]) +  self.drone.rotation_matrix @  np.array([0,0,-self.drone.T]))
        dx[6:9] += self.drone.transformation_matrix @ dx[6:9] 
        dx[9:12] +=  (self.drone.M - np.cross(self.drone.w, self.drone.I @ self.drone.w))
        # print(f"{self.drone.T} and u {self.drone.u}")
        return dx

