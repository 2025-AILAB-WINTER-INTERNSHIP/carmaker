# A single feature point in base_link Cartesian coordinates.
# Extracted from a BEV-transformed segmentation class image.
# Covariance is anisotropic: larger along the radial direction (r) for distant points.

float32 x          # X coordinate in base_link frame [m]
float32 y          # Y coordinate in base_link frame [m]
float32 cov_xx     # Covariance Sigma(0,0) [m^2]
float32 cov_xy     # Covariance Sigma(0,1) = Sigma(1,0) [m^2]
float32 cov_yy     # Covariance Sigma(1,1) [m^2]
uint8   class_id   # 0=background, 1=lane, 2=landmark
