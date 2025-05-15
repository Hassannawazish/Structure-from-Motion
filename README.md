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

The dataset used in this project is taken from  https://www.maths.lth.se/matematiklth/personal/calle/dataset/dataset.html named as Gustav II Adolf. Images and Intrinsic parameters of the camera are given, which include the focal length and principal point. 
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
The sample Image of the Data is shown below.


![Image description](Dataset/statue/DSC_0407.JPG)

### 2. Feature Detection & Matching

Feature detection method(SIFT) is used to identify key points across images. Matching these features between images helps establish correspondences. Feature detection and matching establish reliable 2D–2D correspondences between image pairs. These correspondences are the foundation for estimating relative camera poses (PnP, essential matrix) and for triangulating 3D scene points.

## Feature Detection & Matching: Mathematical Foundations

This section describes the core mathematical steps behind the SIFT-based feature detector, descriptor, matching with Lowe’s ratio test, and RANSAC-based Fundamental matrix outlier rejection as implemented in:


1. **Keypoint Detection (SIFT)**  
   - Build a Gaussian pyramid \(G(x,y,\sigma)\).  
   - Compute Difference‐of‐Gaussians:  
     ```
     DoG(x, y, σ) = G(x, y, kσ) – G(x, y, σ)
     ```  
   - Detect local extrema in the 3×3×3 neighborhood across scale and space.

2. **Descriptor Extraction**  
   - Around each keypoint, form a 16×16 window of gradients.  
   - Divide into a 4×4 grid; in each cell, compute an 8-bin histogram of orientations.  
   - Concatenate and normalize to produce a 128-dimensional descriptor.

3. **Descriptor Matching & Lowe’s Ratio Test**  
   - Use a KD-tree (FLANN) to find, for each descriptor **d**, the two nearest neighbors **d₁**, **d₂** in the second image.  
   - Retain the match only if:  
     ```
     ‖d – d₁‖ / ‖d – d₂‖ < 0.7
     ```

4. **RANSAC + Fundamental Matrix Outlier Rejection**  
   - Given candidate matches \((x₁ⁱ, x₂ⁱ)\), fit the Fundamental matrix **F** under RANSAC (threshold = 1 px, confidence = 0.99).  
   - Enforce the epipolar constraint for inliers:  
     ```
     x₂ⁱᵀ · F · x₁ⁱ = 0
     ```  
   - Keep only those matches that satisfy this within the threshold.

---

### Key Mathematical Concepts

- **Difference‐of‐Gaussians (DoG):**  
  \[
    \mathrm{DoG}(x,y,\sigma) = G(x,y,k\sigma) - G(x,y,\sigma)
  \]  
  Locates scale‐space extrema for invariant keypoint detection.

- **Descriptor Distance Ratio:**  
  \[
    \frac{\|d - d₁\|}{\|d - d₂\|} < 0.7
  \]  
  Filters out ambiguous matches by comparing nearest‐neighbor distances.

- **Epipolar Constraint & Fundamental Matrix:**  
  \[
    x₂ᵀ F x₁ = 0
  \]  
  Governs the relation between corresponding points in two uncalibrated views.

- **RANSAC:**  
  Robustly estimates **F** by repeatedly sampling minimal subsets (8 points), solving for **F**, and selecting the model with the most inliers.

---

### References

- D. G. Lowe, “Distinctive Image Features from Scale-Invariant Keypoints,” *IJCV*, 2004.  
- R. Hartley & A. Zisserman, *Multiple View Geometry in Computer Vision*, Cambridge University Press, 2004.  
- M. A. Fischler & R. C. Bolles, “RANSAC: A Paradigm for Model Fitting,” *CACM*, 1981.  
- OpenCV Docs: https://docs.opencv.org  


### 3. Initial Reconstruction

The first step in reconstruction is triangulation of 3D points from an initial pair of images. This is typically done by solving for the essential matrix \(E\), which relates the two views:

\[
E = K^T F K
\]

### 4. Incremental Camera Registration (PnP)

The Pose-n-Point (PnP) problem is used to estimate the pose of each new image added to the reconstruction. The goal is to minimize the re-projection error:

\[
\min_{R,t} \sum_i \| x_i - \pi(K[R|t]X_i) \|^2
\]

Where \(x_i\) are the 2D projections and \(X_i\) are the 3D points.

### 5. Triangulation & Bundle Adjustment

Triangulation is used to compute the 3D locations of points from corresponding 2D image points. Bundle Adjustment (BA) refines camera poses and 3D points by minimizing the overall re-projection error:

\[
\min_{R_i, t_i, X_j} \sum_{i,j} \| x_{ij} - \pi(K[R_i X_j + t_i]) \|^2
\]

### 6. Colorization (Optional)

To enhance the visualization of the 3D model, RGB values can be assigned to the 3D points by projecting them back to the original images and averaging the colors:

\[
x_i = \pi(K[R_i|t_i]X)
\]

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

# For Custom Dataset 
## Theory

### 1. Camera Calibration

Camera calibration is essential for determining the intrinsic parameters of the camera, which include the focal length and principal point. The intrinsic matrix **K** is defined as:

\[
K = \begin{bmatrix}
f & 0 & c_x \\
0 & f & c_y \\
0 & 0 & 1
\end{bmatrix}
\]

Where:
- \(f\) is the focal length,
- \(c_x, c_y\) are the coordinates of the principal point.

### 2. Feature Detection & Matching

Feature detection methods like SIFT and ORB are used to identify key points across images. Matching these features between images helps establish correspondences. The mathematical formulation of the Fundamental Matrix \(F\) is:

\[
x_j^T F x_i = 0
\]

Where \(x_i\) and \(x_j\) are corresponding points from two different images.

### 3. Initial Reconstruction

The first step in reconstruction is triangulation of 3D points from an initial pair of images. This is typically done by solving for the essential matrix \(E\), which relates the two views:

\[
E = K^T F K
\]

### 4. Incremental Camera Registration (PnP)

The Pose-n-Point (PnP) problem is used to estimate the pose of each new image added to the reconstruction. The goal is to minimize the re-projection error:

\[
\min_{R,t} \sum_i \| x_i - \pi(K[R|t]X_i) \|^2
\]

Where \(x_i\) are the 2D projections and \(X_i\) are the 3D points.

### 5. Triangulation & Bundle Adjustment

Triangulation is used to compute the 3D locations of points from corresponding 2D image points. Bundle Adjustment (BA) refines camera poses and 3D points by minimizing the overall re-projection error:

\[
\min_{R_i, t_i, X_j} \sum_{i,j} \| x_{ij} - \pi(K[R_i X_j + t_i]) \|^2
\]

### 6. Colorization (Optional)

To enhance the visualization of the 3D model, RGB values can be assigned to the 3D points by projecting them back to the original images and averaging the colors:

\[
x_i = \pi(K[R_i|t_i]X)
\]

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
