import os
import numpy as np

def init_ImageLoader(self, img_dir:str, downscale_factor:float):
    """
    Initializes the ImageLoader.

    Args:
        img_dir (str): Directory containing images + 'K.txt' file with intrinsics.
        downscale_factor (float): Factor to downscale images and their intrinsics.
    """
    # ensure directory path is absolute and consistent 
    self.img_dir = os.path.abspath(img_dir)

    #load camera intrinsic matrix from a text file named 'K.txt'
    k_file = os.path.join(self.img_dir, 'K.txt')
    with open(k_file, 'r') as f:
        lines = f.read().strip().split('\n')
        matrix_values = []
        for line in lines:
            row_vals = [float(val) for val in line.strip().split()]
            matrix_values.append(row_vals)
        self.K = np.array(matrix_values, dtype=np.float32) #3x3 

    # collect image file paths
    self.image_list = []
    for filename in sorted(os.listdir(self.img_dir)):
        if filename.lower().endswith(('.jpg', '.jpeg','.png')):
            self.image_list.append(os.path.join(self.img_dir, filename))

    # store the downscale factor and the current working directory 
    self.path = os.getcwd()
    self.factor = downscale_factor

    # adjust the intrinsic matrix for the downscaled image
    self.downscale_instrinsics()
