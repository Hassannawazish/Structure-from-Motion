import cv2
import numpy as np
from matplotlib import pyplot as plt
import os 
from tqdm import tqdm

def sfm_call(self, bundle_adjustment_enabled: bool = True):
    """
    The main entry point to run the entire SfM pipeline.
    """
    pose_array = self.img_obj.K.ravel()
    transform_matrix_0 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
    transform_matrix_1 = np.empty((3,4))
    print('Camera Intrinsic Matrix:', self.img_obj.K)

    pose_0 = np.matmul(self.img_obj.K, transform_matrix_0)
    pose_1 = np.empty((3,4))
    total_points = np.zeros((1,3))
    total_colors = np.zeros((1,3))

    image_0 = self.img_obj.downscale_image(cv2.imread(self.img_obj.image_list[0]))
    image_1 = self.img_obj.downscale_image(cv2.imread(self.img_obj.image_list[1]))

    features_0, features_1 = self.feature_matching(image_0, image_1)

    essential_matrix, em_mask = cv2.findEssentialMat(
        features_0, features_1, self.img_obj.K,
        method=cv2.RANSAC, prob=0.999, threshold=0.4
    )
    if em_mask is not None:
        features_0 = features_0[em_mask.ravel()==1]
        features_1 = features_1[em_mask.ravel()==1]

    _, rot_matrix, tran_matrix, em_mask = cv2.recoverPose(
        essential_matrix, features_0, features_1, self.img_obj.K
    )
    if em_mask is not None:
        features_0 = features_0[em_mask.ravel()>0]
        features_1 = features_1[em_mask.ravel()>0]

    transform_matrix_1[:3,:3] = np.matmul(rot_matrix, transform_matrix_0[:3,:3])
    transform_matrix_1[:3,3] = transform_matrix_0[:3,3] + np.matmul(transform_matrix_0[:3,:3], tran_matrix.ravel())

    pose_1 = np.matmul(self.img_obj.K, transform_matrix_1)

    features_0, features_1, points_3d = self.triangulation(
        pose_0, pose_1, features_0, features_1
    )

    error, points_3d = self.reproj_error(
        points_3d, features_1, transform_matrix_1,
        self.img_obj.K, homogenity=1
    )
    print("Reprojection error for first two images:", error)

    _, _, features_1, points_3d, _ = self.solve_PnP(
        points_3d, features_1, self.img_obj.K,
        np.zeros((5,1), dtype=np.float32),
        features_0, initial=1
    )

    total_images = len(self.img_obj.image_list) - 2
    print('total_images', total_images)

    pose_array = np.hstack((pose_array, pose_0.ravel(), pose_1.ravel()))

    threshold = 0.75

    for i in tqdm(range(total_images)):
        image_2 = self.img_obj.downscale_image(cv2.imread(self.img_obj.image_list[i+2]))
        features_cur, features_2 = self.feature_matching(image_1, image_2)

        if i != 0:
            try:
                features_0, features_1, points_3d = self.triangulation(
                    pose_0, pose_1, features_0, features_1
                )
            except cv2.error as e:
                print(f"⚠️ Skipping image {self.img_obj.image_list[i+2]} (triangulation error: {e})")
                continue

            features_1 = features_1.T
            points_3d = cv2.convertPointsFromHomogeneous(points_3d.T)

        cm_points_0, cm_points_1, cm_mask_0, cm_mask_1 = self.find_common_points(
            features_1, features_cur, features_2
        )
        cm_points_2 = features_2[cm_points_1]
        cm_points_cur = features_cur[cm_points_1]

        rot_matrix, tran_matrix, cm_points_2, points_3d, cm_points_cur = self.solve_PnP(
            points_3d[cm_points_0], cm_points_2, self.img_obj.K,
            np.zeros((5,1), dtype=np.float32), cm_points_cur, initial=0
        )
        transform_matrix_1 = np.hstack((rot_matrix, tran_matrix))
        pose_2 = np.matmul(self.img_obj.K, transform_matrix_1)

        error, points_3d = self.reproj_error(
            points_3d, cm_points_2, transform_matrix_1,
            self.img_obj.K, homogenity=0
        )

        try:
            cm_mask_0, cm_mask_1, points_3d = self.triangulation(
                pose_1, pose_2, cm_mask_0, cm_mask_1
            )
        except cv2.error as e:
            print(f"⚠️ Skipping image {self.img_obj.image_list[i+2]} (triangulation error: {e})")
            continue

        error, points_3d = self.reproj_error(
            points_3d, cm_mask_1, transform_matrix_1,
            self.img_obj.K, homogenity=1
        )
        print("Reprojection error:", error)
        pose_array = np.hstack((pose_array, pose_2.ravel()))

        if bundle_adjustment_enabled:
            points_3d, cm_mask_1, transform_matrix_1 = self.compute_bundle_adjustment(
                points_3d, cm_mask_1, transform_matrix_1,
                self.img_obj.K, threshold
            )
            pose_2 = np.matmul(self.img_obj.K, transform_matrix_1)
            error, points_3d = self.reproj_error(
                points_3d, cm_mask_1,
                transform_matrix_1, self.img_obj.K, homogenity=0
            )
            print("Reprojection error after Bundle Adjustment: ", error)

            total_points = np.vstack((total_points, points_3d))
            points_left = np.array(cm_mask_1, dtype=np.int32)

            try:
                color_vector = np.array([image_2[l[1], l[0]] for l in points_left])
            except Exception as e:
                print(f"⚠️ Skipping image {self.img_obj.image_list[i+2]} (color indexing error: {e})")
                continue
            total_colors = np.vstack((total_colors, color_vector))
        else:
            total_points = np.vstack((total_points, points_3d[:,0,:]))
            points_left = np.array(cm_mask_1, dtype=np.int32)

            try:
                color_vector = np.array([image_2[l[1], l[0]] for l in points_left.T])
            except Exception as e:
                print(f"⚠️ Skipping image {self.img_obj.image_list[i+2]} (color indexing error: {e})")
                continue
            total_colors = np.vstack((total_colors, color_vector))

        transform_matrix_0 = np.copy(transform_matrix_1)
        pose_0 = np.copy(pose_1)
        plt.scatter(i, error)
        plt.pause(0.05)

        image_0 = np.copy(image_1)
        image_1 = np.copy(image_2)
        features_0 = np.copy(features_cur)
        features_1 = np.copy(features_2)
        pose_1 = np.copy(pose_2)

        cv2.imshow(self.img_obj.image_list[0].split('\\')[-2], image_2)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
    cv2.destroyAllWindows()

    plot_dir = os.path.join(
        self.img_obj.path,
        'Results with Bundle Adjustment' if bundle_adjustment_enabled else 'Results'
    )
    if not os.path.exists(plot_dir):
        os.makedirs(plot_dir)

    plt.xlabel('Image Index')
    plt.ylabel('Reprojection Error')
    plt.title('Reprojection Error Plot')
    plt.savefig(os.path.join(plot_dir, 'reprojection_errors.png'))
    plt.close()

    if total_points.size == 0 or total_colors.size == 0:
        print("Error: No points or colors to save. Skipping point cloud generation.")
    else:
        print(f"Total points to save: {total_points.shape[0]}")
        print(f"Total colors to save: {total_colors.shape[0]}")
    scaling_factor = 5000.0
    self.save_to_ply(
        self.img_obj.path, total_points, total_colors,
        bundle_adjustment_enabled, binary_format=True,
        scaling_factor=scaling_factor
    )
    print("Saved the point cloud to .ply file!!!")

    results_dir = os.path.join(self.img_obj.path, 'Results Array')
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    parent_folder = os.path.basename(os.path.dirname(self.img_obj.image_list[0]))
    pose_csv_name = f"{parent_folder}_pose_array.csv"
    pose_csv_path = os.path.join(results_dir, pose_csv_name)
    np.savetxt(pose_csv_path, pose_array, delimiter='\n')
