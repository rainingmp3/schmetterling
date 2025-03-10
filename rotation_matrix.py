import numpy as np

class RotationMatrix:
    def __init__(self, parent):
        self.parent = parent
    
    def get_rotation_matrix(self):
        phi, theta, psi = self.parent.eule
        
        R = np.array([
            [np.cos(theta) * np.cos(psi), -np.cos(theta) * np.sin(psi), np.sin(theta)],
            [np.cos(phi) * np.sin(psi) + np.sin(phi) * np.sin(theta) * np.cos(psi),
             np.cos(phi) * np.cos(psi) - np.sin(phi) * np.sin(theta) * np.sin(psi),
             -np.sin(phi) * np.cos(theta)],
            [np.sin(phi) * np.sin(psi) - np.cos(phi) * np.sin(theta) * np.cos(psi),
             np.sin(phi) * np.cos(psi) + np.cos(phi) * np.sin(theta) * np.sin(psi),
             np.cos(phi) * np.cos(theta)]
        ])
        return R
