# RGB-D Reconstruction and Volume Estimation

This folder contains the English version of the MATLAB code. Comments, console messages, warnings, figure text, table labels, and measurement statuses have been translated. Existing script names and result paths were kept unchanged so that the pipeline remains compatible with the current project structure.

## Required input folders

Place the scripts either in the project root or one level below it, for example in `reconstruction_volume`. The pipeline locates the project root automatically. It expects:

```text
raw/eval/camera_color_image_raw/*.png
raw/eval/camera_depth_image_raw/*.png
```

The RGB and depth filenames must include their timestamps.

## Main execution order

Run the following scripts from MATLAB in this order:

1. `slam_rgbd.m`
   - Synchronizes the RGB and depth frames.
   - Estimates the camera trajectory with RGB-D SLAM.
   - Creates `results/rgbd_slam/slam_poses.mat`.

2. `refinar_poses_hibridas.m`
   - Validates consecutive poses using SIFT, 3D RANSAC, dense depth consistency, and ICP.
   - Keeps reliable SLAM poses and replaces only suspicious transformations that pass the geometric checks.
   - Creates `results/rgbd_slam_hibrido/slam_poses.mat`.

3. `fusion_rgbd.m`
   - Reconstructs every keyframe as a coloured point cloud.
   - Fuses points in a global voxel grid.
   - Keeps voxels observed by at least two keyframes in the consistent map.
   - Creates `results/fusion_rgbd_hibrida/mapa_consistente.ply`.

4. `medir_extintor_rgbd.m`
   - Locates the fire extinguisher from persistent red points.
   - Segments it in each keyframe and reconstructs its 3D points.
   - Rejects partial or inconsistent views.
   - Estimates height, diameter, cylindrical volume, and percentage error relative to the 10 L reference.

`rectificar_rgb_racional.m` is a helper function used automatically by the main scripts. Do not run it directly.

## MATLAB requirements

- Computer Vision Toolbox
- Navigation Toolbox
- A MATLAB version that provides `rgbdvslam`

If `slam_poses.mat` is missing, start again from `slam_rgbd.m`; it is an output of the first stage, not an input that must be downloaded separately.
