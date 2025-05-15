import cv2
import numpy as np

def solve_PnP(self, obj_point, image_point, K, dist_coeff, rot_vector, initial) -> tuple:
    """
    Solves a Perspective-n-Point problem (PnP) via RANSAC to get camera pose.

    Args:
        obj_point (np.ndarray): Nx(1)x3 or Nx3 array of 3D points.
        image_point (np.ndarray): Nx(1)x2 or Nx2 array of 2D points.
        K (np.ndarray): 3x3 camera intrinsic matrix.
        dist_coeff (np.ndarray): Distortion coefficients array.
        rot_vector (np.ndarray): Some initialization or leftover from pipeline.
        initial (int): If 1, modifies shapes and transposes arrays.

    Returns:
        (np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray):
            - rot_matrix: 3x3 rotation matrix
            - tran_vector: 3x1 translation
            - image_point: Possibly subset of 2D inliers
            - obj_point: Possibly subset of 3D inliers
            - rot_vector: Possibly subset of rod vector?
    """
    if initial == 1:
        obj_point = obj_point[:,0,:]
        image_point = image_point.T
        rot_vector = rot_vector.T

    _, rot_vector_calc, tran_vector, inlier = cv2.solvePnPRansac(
        obj_point, image_point, K, dist_coeff, cv2.SOLVEPNP_ITERATIVE
    )
    rot_matrix, _ = cv2.Rodrigues(rot_vector_calc)

    if inlier is not None:
        image_point = image_point[inlier[:,0]]
        obj_point = obj_point[inlier[:,0]]
        rot_vector = rot_vector[inlier[:,0]]
    return rot_matrix, tran_vector, image_point, obj_point, rot_vector
