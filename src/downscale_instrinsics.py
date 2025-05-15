def downscale_instrinsics(self) -> None:
    """
    Adjusts the camera intrinsic parameters to match the downscaled image size.
    """
    self.K[0, 0] /= self.factor #fx
    self.K[1, 1] /= self.factor #fy
    self.K[0, 2] /= self.factor #cx
    self.K[1, 2] /= self.factor #cy
