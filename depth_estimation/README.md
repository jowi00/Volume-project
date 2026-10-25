
# Depth Estimation — team member 1

## Responsibilities
1. Load stereo calibration parameters.
2. Rectify left and right stereo images.
3. Compute disparity using StereoSGBM.
4. Convert disparity to metric depth.
5. Filter invalid depth values and noise.
6. Test using different recordings.

## Expected Outputs
- Rectified stereo images
- Disparity maps
- Depth maps (meters)
- Camera calibration parameters

## Integration
The depth maps will be used by the segmentation
and 3D reconstruction modules.
