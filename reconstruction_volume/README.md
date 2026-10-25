
# 3D Reconstruction and Volume Estimation — team member 3

## Objective
Reconstruct the tank in 3D and estimate its volume
using depth maps and segmentation results.

## Responsibilities
1. Generate a 3D point cloud from segmented depth.
2. Remove noise and outliers.
3. Reconstruct the tank's 3D geometry.
4. Estimate the tank volume in liters.
5. Compare with the reference volume (~10 L).
6. Calculate absolute percentage volume error.

## Inputs
- Depth map from Person 1
- Binary segmentation mask from Person 2
- Camera calibration parameters

## Outputs
- Raw 3D point cloud
- Cleaned 3D point cloud
- Reconstructed tank geometry
- Estimated volume in liters
- Percentage volume error
- Evaluation figures

## Volume Evaluation
Reference volume: 10 liters

Error (%) = abs(Estimated volume - 10) / 10 * 100

Target: Error <= 30%
