import cv2
import numpy as np
from scipy.optimize import least_squares  # though not used here directly

def optimize_reproj_error(self, obj_points) -> np.array:
    """
    Cost function for bundle adjustment used by least_squares.

    Args:
        obj_points (np.ndarray): Flattened array of parameters:
            [3x4 transform_matrix, 3x3 K, 2D points, 3D points, etc.]

    Returns:
        (np.ndarray): Flattened reprojection error array for each point.
    """
    transform_matrix = obj_points[0:12].reshape((3,4))
    K = obj_points[12:21].reshape((3,3))
    rest = int(len(obj_points[21:]) * 0.4)
    p = obj_points[21:21 + rest].reshape((2, int(rest/2))).T
    obj_pts = obj_points[21 + rest:].reshape((int(len(obj_points[21 + rest:])/3),3))

    rot_matrix = transform_matrix[:3,:3]
    tran_vector = transform_matrix[:3, 3]
    rot_vector , _ = cv2.Rodrigues(rot_matrix)

    image_points, _ = cv2.projectPoints(obj_pts, rot_vector, tran_vector, K, None)
    image_points = image_points[:,0,:]
    
    error = [(p[idx]- image_points[idx])**2 for idx in range(len(p))]
    return np.array(error).ravel()/len(p)
