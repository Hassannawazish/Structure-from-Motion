from image_loader import ImageLoader

def init_StructurefromMotion(self, img_dir=str, downscale_factor:float = 2.0):
    """
    Initializes the StructurefromMotion pipeline.

    Args:
        img_dir (str): Directory containing images & K.txt intrinsics.
        downscale_factor (float): Factor used to downscale images & intrinsics.
    """
    # Create an ImageLoader instance
    self.img_obj = ImageLoader(img_dir, downscale_factor)
