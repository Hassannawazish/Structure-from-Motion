import os
import numpy as np
import cv2

def save_to_ply(self, path, point_cloud, colors=None, bundle_adjustment_enabled=False,
                binary_format=False, scaling_factor=1.0, images=None, camera_matrices=None, rotations=None, translations=None):
    """
    Saves the reconstructed 3D point cloud (and optional colors) to a PLY file.

    1) Creates a directory: 'Results' or 'Results with Bundle Adjustment'
    2) Normalizes & optionally outlier-removes the point cloud
    3) Projects RGB values onto 3D points and emphasizes red
    4) Writes ASCII or binary PLY depending on 'binary_format'.
    """
    # Create output directory
    sub_dir = 'Results with Bundle Adjustment' if bundle_adjustment_enabled else 'Results'
    output_dir = os.path.join(path, sub_dir)
    os.makedirs(output_dir, exist_ok=True)

    # Generate PLY filename based on dataset
    dataset_name = os.path.basename(os.path.normpath(self.img_obj.img_dir))
    ply_filename = os.path.join(output_dir, f"{dataset_name}.ply")

    # validate input arrays & scale if needed
    point_cloud = np.asarray(point_cloud).reshape(-1, 3) * scaling_factor

    # Initialize colors as empty if not provided
    if colors is None:
        colors = np.zeros((point_cloud.shape[0], 3), dtype=np.uint8)

    # Project 3D points to 2D and sample RGB values from the images
    if images is not None and camera_matrices is not None and rotations is not None and translations is not None:
        for i, (image, K, R, t) in enumerate(zip(images, camera_matrices, rotations, translations)):
            # Project the 3D points onto the image
            image_points, _ = cv2.projectPoints(point_cloud, R, t, K, distCoeffs=None)

            # Sample RGB values from the 2D projection
            for j, (x, y) in enumerate(image_points[:, 0]):
                x = int(np.round(x))
                y = int(np.round(y))

                # Ensure the point is within the image bounds
                if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                    color = image[y, x]  # Get the RGB value from the image
                    # Increase red channel weight (e.g., double the red)
                    color[2] = min(color[2] * 2, 255)  # Ensure red is boosted but not exceeding 255
                    colors[j] += color  # Accumulate the color values

        # Average the colors across all views
        colors = np.clip(colors, 0, 255).astype(np.uint8)
        colors //= len(images)  # Average the accumulated colors

    # Normalize the point cloud 
    mean = np.mean(point_cloud, axis=0)
    point_cloud -= mean
    scale_factor = np.max(np.linalg.norm(point_cloud, axis=1))
    point_cloud /= scale_factor 

    # Optional outlier removal
    distances = np.linalg.norm(point_cloud, axis=1)
    z_scores = (distances - np.mean(distances)) / np.std(distances)
    mask = np.abs(z_scores) < 2.5
    point_cloud = point_cloud[mask] * scale_factor
    colors = colors[mask]

    # Combine points and colors
    vertices = np.hstack([point_cloud, colors])

    # Write the PLY file
    with open(ply_filename, 'wb' if binary_format else 'w') as f:
        # Header
        if binary_format:
            f.write(b'ply\nformat binary_little_endian 1.0\n')
        else:
            f.write(b'ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(vertices)}\n'.encode())
        f.write(b'property float x\nproperty float y\nproperty float z\n')
        f.write(b'property uchar red\nproperty uchar green\nproperty uchar blue\n')
        f.write(b'end_header\n')

        if binary_format:
            dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                     ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
            vertices_binary = np.zeros((len(vertices),), dtype=dtype)
            vertices_binary['x'] = vertices[:, 0]
            vertices_binary['y'] = vertices[:, 1]
            vertices_binary['z'] = vertices[:, 2]
            vertices_binary['red'] = vertices[:, 3]
            vertices_binary['green'] = vertices[:, 4]
            vertices_binary['blue'] = vertices[:, 5]
            vertices_binary.tofile(f)
        else:
            np.savetxt(f, vertices, fmt='%f %f %f %d %d %d')

    print(f'Point cloud saved to {ply_filename}')
