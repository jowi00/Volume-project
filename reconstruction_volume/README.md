# 3D Reconstruction and Volume Estimation — Team Member 3

## Objective

The objective of this module is to reconstruct the fire extinguisher in 3D and estimate the volume of its tank. The reconstruction uses depth information, camera calibration, object segmentation and camera poses obtained from multiple frames.

The reference tank volume used for evaluation is 10 L.

## Inputs

The module receives:

- Depth maps produced by the depth estimation module
- RGB images and binary masks produced by the segmentation module
- Camera intrinsic parameters
- RGB-D frame timestamps
- Camera poses estimated by the SLAM pipeline

The current reconstruction was tested using the registered RGB-D depth images provided with the recordings. Stereo depth maps can be introduced at the same 3D projection stage.

## Reconstruction Pipeline

### 1. RGB-D Synchronization

RGB and depth images are paired using their timestamps. A maximum synchronization difference of 30 ms is allowed.

For the evaluation sequence:

- 162 RGB images were available
- 85 depth images were available
- 67 synchronized RGB-D pairs were selected
- The median synchronization difference was 4.277 ms

### 2. Camera Pose Estimation

The synchronized frames are processed using RGB-D SLAM. The SLAM system estimates the camera trajectory and selects the keyframes used for reconstruction.

A total of 66 keyframes with valid camera poses were obtained.

### 3. Hybrid Pose Refinement

The SLAM poses are checked using 3D feature correspondences between adjacent keyframes.

The refinement stage uses:

- Feature matching between RGB images
- 3D RANSAC for relative pose estimation
- Spatial coverage checks
- ICP refinement when the initial transformation is reliable
- Geometric validation against the depth maps

A correction is only accepted when it improves the geometric agreement. Otherwise, the original SLAM pose is preserved.

### 4. Multi-view Point Cloud Fusion

Each valid depth pixel is converted into a 3D point using the camera intrinsics:

\[
X = \frac{(u-c_x)Z}{f_x}
\]

\[
Y = \frac{(v-c_y)Z}{f_y}
\]

\[
Z = \text{depth}
\]

The points are transformed from the camera coordinate system to the global coordinate system using the estimated camera pose.

The global point cloud is divided into 4 cm voxels. Points observed from at least two different keyframes are kept in the consistent map.

The final fusion produced:

- 1,817,470 points before voxel fusion
- 728,738 voxels
- 264,717 voxels observed from at least two views
- 137,676 voxels observed from at least three views
- 36.3% multi-view coverage

### 5. Tank Localization and Segmentation

The fire extinguisher is located using its red colour and its spatial consistency across the reconstructed map.

The detected global position is used to select the corresponding tank points in every keyframe. Depth limits and spatial constraints are applied to remove background points and unrelated red objects.

### 6. Dimension Estimation

Principal Component Analysis (PCA) is used to determine the main orientation of the segmented tank.

The tank height and diameter are measured along its principal axes. Robust percentiles are used instead of the minimum and maximum coordinates to reduce the effect of isolated points.

The diameter measurement preserves the sparse points located near the silhouette of the tank, preventing the tank width from being underestimated.

### 7. Multi-view Filtering

Not every keyframe provides a complete and reliable observation of the tank. The final method rejects views with:

- Incomplete tank height
- Unrealistic dimensions
- Inconsistent diameter
- Volume outliers

The remaining measurements are combined using their median, which makes the result less dependent on a single camera view.

## Volume Estimation

The tank is approximated by a cylinder:

\[
V = \pi \left(\frac{D}{2}\right)^2 H
\]

where:

- \(H\) is the estimated tank height
- \(D\) is the estimated tank diameter
- \(V\) is converted from cubic metres to litres

## Results

| Method | Height | Diameter | Volume | Absolute error |
|---|---:|---:|---:|---:|
| Original baseline | 50.2 cm | 13.7 cm | 7.38 L | 26.2% |
| Robust multi-view method | 59.9 cm | 15.1 cm | 10.79 L | 7.9% |

The robust method used 20 consistent keyframes. The interquartile range of the individual volume measurements was 9.81–11.69 L.

The final result is within the required error limit of ±30%.

## Volume Evaluation

The absolute percentage volume error is calculated as:

\[
\text{Error}(\%) =
\frac{\left|V_{\text{estimated}}-V_{\text{reference}}\right|}
{V_{\text{reference}}}
\times 100
\]

Using a reference volume of 10 L:

\[
\text{Error} =
\frac{|10.79-10|}{10}\times100 = 7.9\%
\]

## Main Scripts

The reconstruction pipeline is executed in this order:

```matlab
slam_rgbd
refinar_poses_hibridas
fusion_rgbd
medir_extintor_rgbd
comparar_metodos_extintor
