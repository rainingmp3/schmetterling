import numpy as np

class RotationMatrix:
    """Returns rotation matrix"""
    def __init__(self, parent):
        self.parent = parent
        
    def get_rotation_matrix(self, T=False):
        phi, theta, psi = self.parent.eule
        C_phi = np.cos(phi)
        C_theta = np.cos(theta)
        C_psi = np.cos(psi)
        S_phi = np.sin(phi)
        S_theta = np.sin(theta)
        S_psi =  np.sin(psi)
        R = np.array([[C_psi*C_theta, C_psi*S_theta*S_phi - S_psi*C_phi, C_psi*S_theta*C_phi + S_psi*S_phi],
                      [S_psi*C_theta, S_psi*S_theta*S_phi - C_psi*C_phi, S_psi*S_theta*C_phi + C_psi*S_phi],
                      [     -S_theta,                    C_theta*S_phi ,                     C_theta*C_phi],        
                      ])
        return R
