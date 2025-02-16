# ros2_visualSLAM

This repository contains the `kitti_pub` package which publishes KITTI dataset point clouds and images to ROS2 topics.

## Video Demonstration

[![Screencast](Img%20&%20Video/Screencast%20from%202025-02-16%2006-29-09.webm)](Img%20&%20Video/Screencast%20from%202025-02-16%2006-29-09.webm)


## Directory Structure

```
ros2_visualSLAM/
├── data/
│   └── 2011_09_26/
│       └── 2011_09_26_drive_0001_sync/
│           ├── velodyne_points/
│           │   ├── data/         # Binary .bin files
│           │   └── timestamps.txt
│           ├── image_00/         # Gray left images
│           │   └── data/         # .png or .jpg images
│           ├── image_01/         # Gray right images
│           │   └── data/
│           ├── image_02/         # Color left images
│           │   └── data/
│           └── image_03/         # Color right images
├── src/
│   └── kitti_pub/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── include/
│       │   └── kitti_pub/
│       │       └── kitti_pub_node.hpp
│       ├── src/
│       │   └── kitti_pub_node.cpp
│       └── launch/
│           └── kitti_pub.launch.py
└── README.md
```

## Data Files

- Place binary point cloud files (`.bin`) inside `data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/data`.
- Place the timestamps file at `data/2011_09_26/2011_09_26_drive_0001_sync/velodyne_points/timestamps.txt`.
- Place image files (`.jpg` or `.png`) for each camera in the corresponding image directories:
  - `image_00/data` for left gray images.
  - `image_01/data` for right gray images.
  - `image_02/data` for left color images.
  - `image_03/data` for right color images.

## Launching the Node

A launch file is provided in the `launch` folder. To run the node:

1. Build your workspace:
   ```bash
   colcon build --packages-select kitti_pub
   ```

2. Source the workspace:
   ```bash
   source install/setup.bash
   ```

3. Launch the node:
   ```bash
   ros2 launch kitti_pub kitti_pub.launch.py
   ```


Happy SLAM-ing!