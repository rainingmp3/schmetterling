import numpy as np

class RotationMatrices:
    """
    
    RotationMatrices class handles the transformation of vectors between body and inertial frames.
    Uses ZYX Euler angle convention.
    Transformation matrix handles angular velocity conversation between frames.

    """

    def __init__(self, eule:list)-> None:
        self.eule = eule
    """
    Initialize Rotation Matrices
    Args:
        eule: Euler angles in body frame
    """

    def get_rotation_matrix(self) -> np.ndarray:
        """
        
        Compute rotation matrix using ZYX convention.

        """
        phi, theta, psi = self.eule
        C_phi = np.cos(phi)
        C_theta = np.cos(theta)
        C_psi = np.cos(psi)
        S_phi = np.sin(phi)
        S_theta = np.sin(theta)
        S_psi =  np.sin(psi)
        R = np.array([[C_psi*C_theta, C_psi*S_theta*S_phi - S_psi*C_phi, C_psi*S_theta*C_phi + S_psi*S_phi],
                      [S_psi*C_theta, S_psi*S_theta*S_phi + C_psi*C_phi, S_psi*S_theta*C_phi + C_psi*S_phi],
                      [     -S_theta,                    C_theta*S_phi ,                     C_theta*C_phi],        
                      ])
        return R
   
    def get_transformation_matrix(self) -> np.ndarray:
        """
        Compute transformation matrix for angular velocities.
        
        """
        phi, theta, psi = self.eule
        C_phi = np.cos(phi)
        C_theta = np.cos(theta)
        S_phi = np.sin(phi)
        T_theta = np.tan(theta)
        T = np.array([[1, T_theta*S_phi, T_theta*C_phi],
                      [0,         C_phi,        -S_phi],
                      [0, S_phi/C_theta, C_phi/C_theta]])
        return T