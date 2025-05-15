import cv2
import numpy as np

def triangulation(self, proj_matrix_1, proj_matrix_2, pts_2d_1, pts_2d_2) -> tuple:
    """
    Triangulates 3D points from matched 2D points across two projection matrices,
    but first validates via cheirality (positive depth) using recoverPose.
    Falls back to original triangulation if no valid cheirality inliers.

    Args:
        proj_matrix_1 (np.ndarray): 3x4 projection matrix for view 1.
        proj_matrix_2 (np.ndarray): 3x4 projection matrix for view 2.
        pts_2d_1 (np.ndarray): Nx2 array of matched keypoints in the first image.
        pts_2d_2 (np.ndarray): Nx2 array of matched keypoints in the second image.

    Returns:
        (np.ndarray, np.ndarray, np.ndarray):
            - pts1_used.T (2, M) points for view 1 used in final triangulation
            - pts2_used.T (2, M) points for view 2 used in final triangulation
            - Homogeneous 3D points (4, M) scaled so last row is 1.
    """
    # ensure input points are float32 and contiguous
    pts_orig1 = np.ascontiguousarray(pts_2d_1.astype(np.float32))
    pts_orig2 = np.ascontiguousarray(pts_2d_2.astype(np.float32))

    K = self.img_obj.K.astype(np.float32)

    # 1) Essential + RANSAC
    E, em_mask = cv2.findEssentialMat(
        pts_orig1, pts_orig2, K,
        method=cv2.FM_RANSAC, prob=0.999, threshold=1.0
    )
    if em_mask is not None:
        em_inliers = em_mask.ravel() == 1
        pts1 = pts_orig1[em_inliers]
        pts2 = pts_orig2[em_inliers]
    else:
        pts1, pts2 = pts_orig1, pts_orig2

    # 2) recoverPose → cheirality mask
    _, R, t, rp_mask = cv2.recoverPose(E, pts1, pts2, K)
    cheirality = rp_mask.ravel() == 1
    pts1_clean = pts1[cheirality]
    pts2_clean = pts2[cheirality]

    # prepare projection matrices as float32 contiguous
    P1 = np.ascontiguousarray(proj_matrix_1.astype(np.float32))
    # use original proj_matrix_2 if cheirality fails
    P2_cheir = K.dot(np.hstack((R, t))).astype(np.float32)
    P2_orig = np.ascontiguousarray(proj_matrix_2.astype(np.float32))

    # choose points + P2 based on cheirality success
    if pts1_clean.shape[0] >= 2:
        pts1_used = pts1_clean
        pts2_used = pts2_clean
        P2 = P2_cheir
    else:
        # fallback
        pts1_used = pts_orig1
        pts2_used = pts_orig2
        P2 = P2_orig

    # 3) Triangulate: inputs must be 2xN float32 contiguous
    pts1_arr = np.ascontiguousarray(pts1_used.T.astype(np.float32))
    pts2_arr = np.ascontiguousarray(pts2_used.T.astype(np.float32))

    point_cloud = cv2.triangulatePoints(P1, P2, pts1_arr, pts2_arr)
    point_cloud /= point_cloud[3]  # normalize homogeneous

    return pts1_arr, pts2_arr, point_cloud
