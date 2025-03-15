import numpy as np

class StateMatrices:
    """Stores state matrcies ABCD"""
    def __init__(self, parent):
        self.parent = parent
        self.update_matrices()

    def update_matrices(self):
        g = self.parent.g
        mass = self.parent.params['mass']
        I = self.parent.I
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

    def eom(self, states , inputs):
        dx = self.A @ states + self.B @ inputs
        dx[3:6] = dx[3:6] @  self.parent.rotation_matrix 
        dx[6:9] = dx[6:9] @self.parent.transformation_matrix
        return dx

