## How to Run

### 1. Download the project

Download the repository using **Code → Download ZIP** and extract it.

### 2. Add the dataset

Copy the `raw` dataset folder into the project root. The required structure is:

```text
Volume-project/
├── reconstruction_volume/
│   ├── slam_rgbd.m
│   ├── rectificar_rgb_racional.m
│   ├── refinar_poses_hibridas.m
│   └── fusion_rgbd.m
│
└── raw/
    └── eval/
        ├── camera_color_image_raw/
        │   └── *.png
        └── camera_depth_image_raw/
            └── *.png
