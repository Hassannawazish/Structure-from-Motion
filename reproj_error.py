import cv2
import numpy as np

def reproj_error(self, obj_points, image_points, transform_matrix, K, homogenity) -> tuple:
    """
    Computes the reprojection error for given 3D-2D correspondences and a camera transform.

    Args:
        obj_points (np.ndarray): Nx4 (homogeneous) or Nx3 points in 3D.
        image_points (np.ndarray): Nx2 array of 2D points.
        transform_matrix (np.ndarray): 3x4 extrinsic matrix [R | t].
        K (np.ndarray): 3x3 camera intrinsic matrix.
        homogenity (int): If 1, interpret obj_points as homogeneous.

    Returns:
        (float, np.ndarray):
            - total_error / len(image_points_calc): The average reprojection error.
            - obj_points: Possibly converted from homogeneous to Nx3 if homogenity == 1.
    """
    rot_matrix = transform_matrix[:3,:3]
    tran_vector = transform_matrix[:3, 3]
    rot_vector, _ = cv2.Rodrigues(rot_matrix)

    if homogenity == 1:
        obj_points = cv2.convertPointsFromHomogeneous(obj_points.T)

    image_points_calc, _ = cv2.projectPoints(
        obj_points, rot_vector, tran_vector, K, None
    )
    image_points_calc = np.float32(image_points_calc[:,0,:])

    total_error = cv2.norm(
        image_points_calc,
        np.float32(image_points.T) if homogenity == 1 else np.float32(image_points),
        cv2.NORM_L2
    )
    return total_error/ len(image_points_calc), obj_points
