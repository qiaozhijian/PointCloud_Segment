# Point Cloud Plane Segmentation Using PCL library
## Overview
This is a simple project to demonstrate how to use PCL library to segment a point cloud into multiple planes. The project is based on the PCL tutorial. The project is developed using C++ and CMake.

Provided methods:
- RANSAC
- Region Growing // also used for clustering
- RANSAC with normal estimation

## Run
```bash
mkdir build
cd build
cmake ..
make
# modify the configs in configs/params.yaml
./plane_segmentation
```

## Results
Region Growing cannot segment the planes correctly. It's better for clustering.

RANSAC cannot do well, too. Because it just slices the point cloud into several layers.