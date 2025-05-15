import os
import numpy as np

def save_to_ply(self, path, point_cloud, colors=None, bundle_adjustment_enabled=False,
                binary_format=False, scaling_factor=1.0):
    """
    Saves the reconstructed 3D point cloud (and optional colors) to a PLY file.

    1) Creates a directory: 'Results' or 'Results with Bundle Adjustment'
    2) Normalizes & optionally outlier-removes the point cloud
    3) Writes ASCII or binary PLY depending on 'binary_format'.
    """
    # Create output directory
    sub_dir = 'Results with Bundle Adjustment' if bundle_adjustment_enabled else 'Results'
    output_dir = os.path.join(path, sub_dir)
    os.makedirs(output_dir, exist_ok=True)

    # Generate PLY filename based on dataset
    dataset_name = os.path.basename(os.path.normpath(self.img_obj.img_dir))
    ply_filename = os.path.join(output_dir, f"{dataset_name}.ply")

    # validate input arrays & scale if needed
    point_cloud = np.asarray(point_cloud).reshape(-1,3) * scaling_factor

    if colors is not None:
        colors = np.asarray(colors).reshape(-1,3)
        colors = np.clip(colors, 0, 255).astype(np.uint8)
        if colors.shape[0] != point_cloud.shape[0]:
            n = min(colors.shape[0], point_cloud.shape[0])
            colors = colors[:n]
            point_cloud = point_cloud[:n]
    else:
        colors = np.full_like(point_cloud, fill_value=105, dtype=np.uint8)

    # Normalize the point cloud 
    mean = np.mean(point_cloud, axis=0)
    point_cloud -= mean
    scale_factor = np.max(np.linalg.norm(point_cloud, axis=1))
    point_cloud /= scale_factor 

    # optional outlier removal
    distances = np.linalg.norm(point_cloud, axis=1)
    z_scores = (distances - np.mean(distances)) / np.std(distances)
    mask = np.abs(z_scores) < 2.5
    point_cloud = point_cloud[mask] * scale_factor
    colors = colors[mask]

    # combine points and colors
    vertices = np.hstack([point_cloud, colors])

    # write the PLY file
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
            vertices_binary['x'] = vertices[:,0]
            vertices_binary['y'] = vertices[:,1]
            vertices_binary['z'] = vertices[:,2]
            vertices_binary['red'] = vertices[:,3]
            vertices_binary['green'] = vertices[:,4]
            vertices_binary['blue'] = vertices[:,5]
            vertices_binary.tofile(f)
        else:
            np.savetxt(f, vertices, fmt='%f %f %f %d %d %d')

    print(f'Point cloud saved to {ply_filename}')
