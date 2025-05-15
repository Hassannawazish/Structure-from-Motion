# Incremental Structure from Motion (SfM) for 3D Reconstruction

## Project Overview

This project focuses on the development of an incremental Structure from Motion (SfM) pipeline to reconstruct 3D point clouds from image sequences. The project includes camera calibration, feature detection, matching, initial reconstruction, and incremental expansion, aiming to generate an accurate 3D reconstruction.

---

## Table of Contents

1. [Introduction](#introduction)
2. [Theory](#theory)
   - Camera Calibration
   - Feature Detection & Matching
   - Initial Reconstruction
   - Incremental Camera Registration (PnP)
   - Triangulation & Bundle Adjustment
   - Colorization (Optional)
3. [Implementation](#implementation)
4. [Results](#results)
5. [Mathematics](#mathematics)
6. [References](#references)

---

## Introduction

The goal of this project is to reconstruct 3D models from 2D image sequences by estimating the camera poses and 3D geometry incrementally. We use methods like feature detection (e.g., SIFT/ORB), triangulation, bundle adjustment, and camera pose estimation to build a 3D point cloud.

---

# For Existing Dataset 

## Theory

### 1. Dataset Description

The dataset used in this project is taken from [Gustav II Adolf Dataset](https://www.maths.lth.se/matematiklth/personal/calle/dataset/dataset.html). Images and intrinsic parameters of the camera are given, which include the focal length and principal point.  
The intrinsic matrix **K** is defined as:

$$
K = \begin{bmatrix}
f & 0 & c_x \\
0 & f & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

Where:
- \(f\) is the focal length,
- \(c_x, c_y\) are the coordinates of the principal point.  
The sample image of the data is shown below.

![Image description](Dataset/statue/DSC_0407.JPG)

### 2. Feature Detection & Matching

Feature detection methods, particularly SIFT (Scale-Invariant Feature Transform), are used to identify keypoints across multiple images. These keypoints are then matched between the images to establish correspondences. Matching is typically done using a nearest neighbor search with a ratio test to eliminate ambiguous matches.

Mathematically, we can describe the feature matching process as follows:

1. **SIFT Keypoint Detection**:  
   SIFT detects interest points that are invariant to scale, rotation, and partially invariant to affine transformations and illumination changes. The process starts by creating a Gaussian pyramid for each image to detect scale-invariant keypoints. The keypoints are then described using local feature descriptors.

   $$
   \mathcal{D}(x, y, \sigma) = \left[ I(x, y), I(x + \Delta x, y + \Delta y) \dots \right]
   $$

   Where:
   - \( \mathcal{D}(x, y, \sigma) \) represents the descriptor for the point \( (x, y) \) at scale \( \sigma \),
   - \( I(x, y) \) is the intensity at the point \( (x, y) \).

2. **Feature Matching (FLANN-Based Matcher)**:  
   After detecting keypoints, the next step is to match these keypoints between two images. This is done using a FLANN-based matcher (Fast Library for Approximate Nearest Neighbors), which finds the nearest neighbors between the descriptors of the keypoints. FLANN is optimized to perform efficient matching across a large dataset of descriptors.

   The mathematical basis for the matching is defined as:

   $$
   \text{distance} = \| \mathcal{D}(x_1, y_1) - \mathcal{D}(x_2, y_2) \|
   $$

   Here:
   - \( \mathcal{D}(x_1, y_1) \) and \( \mathcal{D}(x_2, y_2) \) represent descriptors of keypoints in two images.

3. **Ratio Test (Lowe's Method)**:  
   Lowe’s ratio test is applied to filter out unreliable matches. The idea is that for each keypoint in the first image, the nearest and second nearest neighbors from the second image should have a ratio of distances that is below a threshold. This helps remove ambiguous matches where multiple keypoints may be close together.

   $$
   \frac{d_1}{d_2} < 0.7
   $$

   Where:
   - \( d_1 \) is the distance to the nearest neighbor,
   - \( d_2 \) is the distance to the second nearest neighbor.

4. **RANSAC and Fundamental Matrix**:  
   RANSAC (Random Sample Consensus) is used to estimate the Fundamental Matrix \( F \) that relates corresponding points in two images. The fundamental matrix can be defined as:

   $$
   x_j^T F x_i = 0
   $$

   Where \( x_i \) and \( x_j \) are corresponding points from two images. RANSAC is used to reject outlier matches by iterating over random samples of point correspondences and computing the fundamental matrix.

   The output of the RANSAC process is a binary mask indicating the inliers (matches that are consistent with the estimated fundamental matrix). The points that correspond to inliers are retained for further processing.

### 3. Initial Reconstruction

The first step in reconstruction is triangulation of 3D points from an initial pair of images. This is typically done by solving for the essential matrix \( E \), which relates the two views:

$$
E = K^T F K
$$

### 4. Incremental Camera Registration (PnP)

The Pose-n-Point (PnP) problem is used to estimate the pose of each new image added to the reconstruction. The goal is to minimize the re-projection error:

$$
\min_{R,t} \sum_i \| x_i - \pi(K[R|t]X_i) \|^2
$$

Where \(x_i\) are the 2D projections and \(X_i\) are the 3D points.

### 5. Triangulation & Bundle Adjustment

Triangulation is used to compute the 3D locations of points from corresponding 2D image points. Bundle Adjustment (BA) refines camera poses and 3D points by minimizing the overall re-projection error:

$$
\min_{R_i, t_i, X_j} \sum_{i,j} \| x_{ij} - \pi(K[R_i X_j + t_i]) \|^2
$$

### 6. Colorization (Optional)

To enhance the visualization of the 3D model, RGB values can be assigned to the 3D points by projecting them back to the original images and averaging the colors:

$$
x_i = \pi(K[R_i|t_i]X)
$$

---

## Implementation

The implementation follows these steps:

1. **Camera Calibration**: Perform using OpenCV functions such as `cv2.calibrateCamera()`.
2. **Feature Detection & Matching**: Use SIFT/ORB and filter outliers using Lowe’s ratio test.
3. **Initial Reconstruction**: Compute the essential matrix and triangulate initial points.
4. **Incremental Expansion**: Add new images, estimate poses using PnP, and perform triangulation.
5. **Bundle Adjustment**: Refine the 3D model and camera poses using optimization techniques.
6. **Colorization (Optional)**: Project 3D points back to the images to assign colors.

---

## Results

### 3D Point Cloud

[Insert 3D Point Cloud Image Here]

- Point Cloud Visualizations
- Camera Trajectories

### Reprojection Error

[Insert Reprojection Error Graph Here]

---

## Mathematics

The following mathematical formulations were used in the implementation:

- Camera Calibration Matrix \( K \)
- Fundamental Matrix \( F \)
- Essential Matrix \( E \)
- PnP Pose Estimation
- Bundle Adjustment Minimization Formula

---

## References

1. Hartley, R., & Zisserman, A. (2004). *Multiple View Geometry in Computer Vision*. Cambridge University Press.
2. OpenCV Documentation: [https://docs.opencv.org/](https://docs.opencv.org/)
3. Middlebury 3D Reconstruction Datasets: [https://vision.middlebury.edu/mview/data/](https://vision.middlebury.edu/mview/data/)
