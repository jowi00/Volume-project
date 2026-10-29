https://ltuse-my.sharepoint.com/personal/christoforos_kanellakis_ltu_se/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fchristoforos%5Fkanellakis%5Fltu%5Fse%2FDocuments%2F%5BR7020E%5D%20%2D%20Projects%2FR7020E%5Fproject%5Fdata%2FProject%5F2&ga=1  

# Guide to execute all the code 

1 - Place the .m files in the root directory:

```
    ├── main.m                         # execute and generate all scripts and figures (execute all the pipeline)
    ├── generate_depth_maps.m          # script to generate depth maps thanks to pair IR images
    ├── parse_camera_yaml.m            # functions for generate_depth_maps.m
    ├── segmentation.m                 # script to generate binary masks and segmented depth maps
    ├── segment_extinguisher.m         # functions for segmentation.m 
    ├── reconstruction.m               # script to generate point cloud of the tank and it cylindral representation
    ├── generate_bounding_box_video.m  # script to generate a video of the tank with it volume estimation for each raw image
    └── Project_2                      # !!! You might not have the same folder name as me !!!
    │   ├── raw
    │   │   ├── ...   
    │   └── rosbags
    │   │   ├── ...
```

2 - Execute main.m. You should obtain (at the root):
```
  - generated_depth_maps/ (depth maps for each pair of IR pictures) (Person 1)
  - binary_masks/ (mask of the tank for each raw images) (Person 2)
  - depth_masked/ (depth maps segmentation with only the tank) (Person 2)
  - Figure: 360° Cylindrical 3D Reconstruction of Fire Extinguisher (In Matlab)
  - extinguisher_volume_tracking.mp4 (video of the tank with it volume estimation for each raw image)
```

3 - There is the main.m that compute all the code, but generate_depth_maps.m, segmentation.m, reconstruction.m and generate_bounding_box_video.m are here to have a better separation of the code.
If you decide first to run each part independly, don't forget that segmentation.m needs results from generate_depth_maps.m, reconstruction.m from segmentation.m and etc..

# Do not forget that, for every person, everyone should master it own part pls
