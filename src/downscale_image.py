import cv2

def downscale_image(self, image):
    """
    Downscales the given image by self.factor using single-step resizing.

    Args:
        image (np.ndarray): BGR image loaded via cv2.imread.

    Returns:
        np.ndarray: Downscaled image.
    """
    #single-step resize
    new_w = int(image.shape[1] / self.factor)
    new_h = int(image.shape[0] / self.factor)
    return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
