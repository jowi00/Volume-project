# Depth Estimation — Team Member 1

## Objective

This module estimates metric depth from the left and right infrared images provided in the recordings. Its output is a filtered depth map that can be used by the segmentation and 3D reconstruction modules.

## Input Data

The module uses:

- Left infrared images
- Right infrared images
- Image timestamps
- Stereo camera calibration
- Camera intrinsic parameters
- Stereo baseline

The images have a resolution of 640 × 400 pixels.

## Processing Pipeline

### 1. Stereo Pair Synchronization

Left and right infrared images are matched using their timestamps. Only synchronized pairs available in both camera streams are processed.

The evaluation recording contains 100 valid stereo pairs.

### 2. Calibration Verification

The camera parameters are read from the calibration information included with the recordings.

The main parameters used for metric depth estimation are:

- Focal length: 306.0002 pixels
- Stereo baseline: 0.100368 m
- Principal point: approximately (318.48, 201.37)

The baseline was verified using the camera transformation stored in the rosbag. This value is used instead of relying on the sign of the right projection matrix, which was not consistent with the observed disparity direction.

### 3. Stereo Rectification

The stereo images are rectified using the camera calibration so that corresponding points are located on the same image row.

This reduces the matching problem to a horizontal search and allows disparity to be calculated as:

\[
d = x_L - x_R
\]

For this recording, valid disparities are positive.

### 4. Disparity Estimation

The disparity map is calculated using Semi-Global Matching. In MATLAB, this is implemented with `disparitySGM`.

The disparity search range is:

\[
0 \leq d \leq 64
\]

GPU processing is used when available to reduce the execution time.

### 5. Conversion from Disparity to Depth

Each valid disparity value is converted into metric depth using:

\[
Z = \frac{fB}{d}
\]

where:

- \(Z\) is the depth in metres
- \(f\) is the focal length in pixels
- \(B\) is the stereo baseline in metres
- \(d\) is the disparity in pixels

A small disparity represents a distant point, while a large disparity represents a point closer to the cameras.

### 6. Invalid-Depth Filtering

Pixels are rejected when:

- The disparity is zero or invalid
- The calculated depth is not finite
- The depth is below 0.5 m
- The depth is above 6 m
- The stereo match is not reliable

The remaining depth values form the final valid-depth mask. Noise filtering is then applied without changing the metric scale.

## Main Parameters

| Parameter | Value |
|---|---:|
| Image resolution | 640 × 400 px |
| Focal length | 306.0002 px |
| Baseline | 0.100368 m |
| Disparity range | 0–64 px |
| Valid depth range | 0.5–6.0 m |
| Disparity direction | \(x_L-x_R\) |

## Test Results

The pipeline was tested on different pairs from the evaluation recording.

| Stereo pair | Valid depth pixels | Median depth |
|---|---:|---:|
| Pair 1 | 58.0% | 2.310 m |
| Pair 50 | 50.7% | 2.059 m |

The GPU disparity calculation required approximately 0.020–0.022 seconds per stereo pair in the tested configuration.

## Sensor-Depth Comparison

The stereo result was also compared with the recorded depth sensor for validation. The sensor depth was not used to calculate the stereo depth map.

For test pair 50:

- Timestamp difference: 4.891 ms
- Common valid pixels: 84,270
- Median stereo-to-sensor depth ratio: 0.8959
- Median absolute depth difference: 0.3156 m

This comparison was used to check the depth scale and identify the remaining stereo estimation error.

## Outputs

This module provides:

- Rectified left and right images
- Disparity map
- Depth map in metres
- Logical valid-depth mask
- Calibration and processing parameters

The segmentation module can apply its binary object mask directly to the depth map. The resulting segmented depth values can then be projected into 3D by the reconstruction module.
