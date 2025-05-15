from image_loader import ImageLoader
from image_loader_init import init_ImageLoader
from downscale_image import downscale_image
from downscale_instrinsics import downscale_instrinsics


# patch ImageLoader
ImageLoader.__init__              = init_ImageLoader
ImageLoader.downscale_image       = downscale_image
ImageLoader.downscale_instrinsics = downscale_instrinsics

from structure_from_motion import StructurefromMotion
from sfm_init import init_StructurefromMotion
from feature_matching import feature_matching
from triangulation import triangulation
from solve_pnp import solve_PnP
from find_common_points import find_common_points
from reproj_error import reproj_error
from optimize_reproj_error import optimize_reproj_error
from compute_bundle_adjustment import compute_bundle_adjustment
from save_to_ply import save_to_ply
from sfm_call import sfm_call
from visualize_point_cloud import visualize_point_cloud

# patch StructurefromMotion
StructurefromMotion.__init__                  = init_StructurefromMotion
StructurefromMotion.feature_matching          = feature_matching
StructurefromMotion.triangulation             = triangulation
StructurefromMotion.solve_PnP                 = solve_PnP
StructurefromMotion.find_common_points        = find_common_points
StructurefromMotion.reproj_error              = reproj_error
StructurefromMotion.optimize_reproj_error     = optimize_reproj_error
StructurefromMotion.compute_bundle_adjustment = compute_bundle_adjustment
StructurefromMotion.save_to_ply               = save_to_ply
StructurefromMotion.__call__                  = sfm_call

if __name__ == '__main__':
    sfm = StructurefromMotion("Dataset/statue")
    sfm()
    sfm(bundle_adjustment_enabled=True)

    ply_path = r"Results with Bundle Adjustment\statue.ply"
    visualize_point_cloud(ply_path)