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

Feature detection method(SIFT) is used to identify key points across images. Matching these features between images helps establish correspondences. 


## Feature Detection & Matching

### Concept Overview  
Feature detection and matching establish reliable 2D–2D correspondences between image pairs. These correspondences are the foundation for estimating relative camera poses (PnP, essential matrix) and for triangulating 3D scene points.

---

### Pipeline Steps

1. **Keypoint Detection (SIFT)**  
   - Build a scale-space pyramid by convolving the image with Gaussians at multiple scales.  
   - Compute Difference-of-Gaussians (DoG) between adjacent scales.  
   - Detect keypoints as local extrema in the 3×3×3 neighborhood across space and scale.  

2. **Descriptor Extraction**  
   - Around each keypoint, sample image gradients in a 16×16 pixel window.  
   - Partition into a 4×4 grid of cells; in each cell, form an 8-bin orientation histogram of gradient directions.  
   - Concatenate histograms into a 128-dimension vector and normalize to unit length.

3. **Descriptor Matching & Lowe’s Ratio Test**  
   - Use a KD-tree (FLANN) to find the two nearest descriptors \(d_1, d_2\) in the second image for each descriptor \(d\) in the first.  
   - Retain the match \(d \leftrightarrow d_1\) only if  
     \[
       \frac{\lVert d - d_1\rVert}{\lVert d - d_2\rVert} < 0.7,
     \]  
     filtering out ambiguous or repeated patterns.

4. **RANSAC + Fundamental Matrix Outlier Rejection**  
   - Estimate the Fundamental matrix \(F\) under RANSAC from matched points \(\{\mathbf{x}_1^i,\mathbf{x}_2^i\}\).  
   - The epipolar constraint  
     \[
       \mathbf{x}_2^{i\top}\,F\,\mathbf{x}_1^i = 0
     \]  
     must hold for inliers.  
   - Iteratively sample minimal subsets (8 points), solve for \(F\) (eight-point algorithm), and count inliers with  
     \(\lvert \mathbf{x}_2^\top F\,\mathbf{x}_1\rvert < 1\) px.  
   - Final inlier set defines the filtered correspondences.

---

### Mathematical Details

#### 1. DoG Keypoint Localization  
- **Gaussian pyramid**:  
  \(\displaystyle G(x,y,\sigma) = I * \mathcal{N}(0,\sigma^2)\)  
- **DoG image**:  
  \(\displaystyle D(x,y,\sigma) = G(x,y,k\sigma) - G(x,y,\sigma)\)  
- **Extrema**:  
  A sample is a keypoint if it is a local maximum or minimum in its 3×3×3 neighborhood in \(D\).

#### 2. SIFT Descriptor Formation  
- Compute gradient magnitude \(m(x,y)\) and orientation \(\theta(x,y)\).  
- In each 4×4 cell, form histogram  
  \[
    h_j = \sum_{\text{cell}} m(x,y)\,\delta\bigl(\theta(x,y)\in\text{bin }j\bigr),\quad j=1\ldots8.
  \]  
- Descriptor vector \(\mathbf{d}\in\mathbb{R}^{128}\) is \([h_1,\dots,h_{128}]^\top\), then normalized.

#### 3. Lowe’s Ratio Test  
- Given distances \(r_1 = \lVert d - d_1\rVert\) and \(r_2 = \lVert d - d_2\rVert\), accept match if  
  \(\,r_1 / r_2 < 0.7.\)

#### 4. Epipolar Geometry & Fundamental Matrix  
- For corresponding homogeneous points \(\mathbf{x}_1, \mathbf{x}_2\):  
  \(\displaystyle \mathbf{x}_2^\top F\,\mathbf{x}_1 = 0.\)  
- \(F\) is estimated (eight-point algorithm) under the rank-2 constraint.

#### 5. RANSAC for Robust Estimation  
1. Randomly sample 8 matches.  
2. Estimate \(F\) and enforce \(\mathrm{rank}(F)=2\).  
3. Count inliers satisfying \(\lvert \mathbf{x}_2^\top F\,\mathbf{x}_1\rvert < 1\).  
4. Repeat and choose the model with the largest inlier set.
5. 

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
