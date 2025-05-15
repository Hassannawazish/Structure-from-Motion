import cv2
import numpy as np
import os

def feature_matching(self, image_0, image_1) -> tuple:
    """
    Detects and matches SIFT keypoints between two images using FLANN-based matcher,
    filters out outliers via RANSAC on the Fundamental matrix,
    saves each matches–image into a “Matches” folder, and displays it for 0.3s.

    Args:
        image_0 (np.ndarray): First input BGR image.
        image_1 (np.ndarray): Second input BGR image.

    Returns:
        (np.ndarray, np.ndarray): Two Nx2 arrays of inlier matched keypoints (pts0, pts1).
    """
    # initialize output folder & counter on first call
    if not hasattr(self, 'match_dir'):
        self.match_dir = os.path.join(os.getcwd(), 'Matches')
        os.makedirs(self.match_dir, exist_ok=True)
        self.match_idx = 0

    # 1) Detect & describe
    sift = cv2.SIFT_create(nfeatures=10000)
    kp0, des0 = sift.detectAndCompute(cv2.cvtColor(image_0, cv2.COLOR_BGR2GRAY), None)
    kp1, des1 = sift.detectAndCompute(cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY), None)

    # 2) Match + ratio test
    flann = cv2.FlannBasedMatcher(dict(algorithm=1, trees=15), dict(checks=200))
    matches = flann.knnMatch(des0, des1, k=2)
    good = [m for m, n in matches if m.distance < 0.7 * n.distance]

    pts0 = np.float32([kp0[m.queryIdx].pt for m in good])
    pts1 = np.float32([kp1[m.trainIdx].pt for m in good])

    # 3) RANSAC filter if possible
    if len(pts0) >= 8:
        F, mask = cv2.findFundamentalMat(pts0, pts1,
                                         method=cv2.FM_RANSAC,
                                         ransacReprojThreshold=1.0,
                                         confidence=0.99)
        mask = mask.ravel()
        inliers0 = pts0[mask == 1]
        inliers1 = pts1[mask == 1]
        inlier_matches = [good[i] for i in range(len(good)) if mask[i] == 1]

        # draw inliers
        img_matches = cv2.drawMatches(
            image_0, kp0, image_1, kp1,
            inlier_matches, None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )
    else:
        # not enough for RANSAC: draw all good matches
        inliers0, inliers1 = pts0, pts1
        img_matches = cv2.drawMatches(
            image_0, kp0, image_1, kp1,
            good, None,
            flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
        )

    # 4) save to disk
    # filename = f"match_{self.match_idx:04d}.png"
    # cv2.imwrite(os.path.join(self.match_dir, filename), img_matches)
    # self.match_idx += 1

    # # 5) display for 0.3 seconds
    # cv2.imshow('Matches', img_matches)
    # cv2.waitKey(300)
    # cv2.destroyAllWindows()

    return inliers0, inliers1
