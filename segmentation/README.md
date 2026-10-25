## Guide to Obtaining binary_mask (2D) and depth_masked (2.5D)
1 - Place the files in the root directory:
    ├── script.m
    ├── segment_extinguisher.m
    └── Project_2         #!!! You might not have the same folder name as me !!!
    │   ├── raw
    │   │   ├── ...   
    │   └── rosbags
    │   │   ├── ...
2 - Execute script.m to obtain binary_masks and depth_masked in the same tree structure
    
    
# Object Segmentation — Person 2

## Responsibilities
1. Analyse the dataset.
2. Implement tank segmentation.
3. Generate binary segmentation masks.
4. Improve segmentation accuracy.
5. Apply masks to depth maps.
6. Evaluate segmentation results.

## Expected Outputs
- Binary tank masks
- Segmented depth maps
- Segmentation visualizations

## Integration
The segmented depth data will be used for 3D reconstruction and
volume estimation.
