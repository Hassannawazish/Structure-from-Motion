import open3d as o3d

def visualize_point_cloud(ply_file_path):
    """
    Load, optionally filter, and visualize a point cloud from a .ply file.
    """
    pcd = o3d.io.read_point_cloud(ply_file_path)
    if pcd.is_empty():
        print("Failed to load point cloud")
        return

    print(f"Loaded point cloud with {len(pcd.points)} points")

    # Optional: filter point cloud
    voxel_size = 0.05  # Tune this if needed
    pcd_down = pcd.voxel_down_sample(voxel_size)
    print(f"📉 Reduced point cloud size: {len(pcd_down.points)}")

    # Visualization
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="SfM Point Cloud Viewer")
    vis.add_geometry(pcd_down)
    vis.get_render_option().point_size = 2.0
    vis.run()
    vis.destroy_window()
