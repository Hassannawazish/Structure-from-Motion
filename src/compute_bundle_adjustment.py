import numpy as np
from scipy.optimize import least_squares

def compute_bundle_adjustment(self, _3d_point, opt, transform_matrix_new, K, r_error) -> tuple:
    """
    Runs a least_squares optimizer to refine camera extrinsics, intrinsics, and 3D points.

    Args:
        _3d_point (np.ndarray): Nx3 array of 3D points.
        opt (np.ndarray): Nx2 array of 2D points.
        transform_matrix_new (np.ndarray): 3x4 extrinsic matrix to be refined.
        K (np.ndarray): 3x3 camera intrinsics to be refined.
        r_error (float): Convergence tolerance for least_squares.

    Returns:
        (np.ndarray, np.ndarray, np.ndarray):
            - Refined 3D points (Nx3).
            - Refined 2D points (2xN).
            - Refined extrinsic matrix (3x4).
    """
    opt_variables = np.hstack((transform_matrix_new.ravel(), K.ravel()))
    opt_variables = np.hstack((opt_variables, opt.ravel()))
    opt_variables = np.hstack((opt_variables, _3d_point.ravel()))

    values_corrected = least_squares(self.optimize_reproj_error, opt_variables, gtol=r_error).x
    
    K_new = values_corrected[12:21].reshape((3,3))
    rest = int(len(values_corrected[21:]) * 0.4)
    pts_2d = values_corrected[21:21 + rest].reshape((2, int(rest/2))).T
    pts_3d = values_corrected[21+rest:].reshape((int(len(values_corrected[21+rest:])/3),3))
    transform_new = values_corrected[0:12].reshape((3,4))

    return pts_3d, pts_2d, transform_new
