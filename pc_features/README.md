
# Point Cloud Feature Filtering with Sobel and SRI

This repository contains a ROS node that filters edges, surfaces, and ground from a LiDAR point cloud by converting it into a spherical range image (SRI). It uses Sobel convolutional filters to extract these features, and then reconstructs them back into a point cloud for further processing in robotic applications.

## Description

The node transforms point clouds from LiDAR sensors into spherical range images (SRI). Sobel operators are applied to the SRI to extract edges, surfaces, and ground features. These filtered features are then reconstructed into a point cloud format.

The Sobel operator computes discrete derivatives to approximate the gradient of an image intensity function by convolving the original image with a Sobel mask.

This feature extraction is used in [LiLO: Lightweight and lowbias LiDAR Odometry method based on spherical range image filtering](https://arxiv.org/abs/2311.07291), a method designed for low-bias LiDAR odometry that uses spherical range images to filter point cloud data.

![image](https://github.com/user-attachments/assets/c3137203-04b6-42be-903c-9aae7f6ff8a1)

The figure shows the feature extraction of the KITTI dataset in sequence 08, using the proposed method and comparing it with the classical feature extraction method tested in LOAM-based methods.

## Requirements

To run this node, the following dependencies are required:

- **[ROS 1](https://wiki.ros.org/noetic/Installation/Ubuntu)** (testing in ROS Noetic)
- **PCL (Point Cloud Library)** for point cloud manipulation
- **Eigen** for linear algebra operations
- **Catkin** to build the ROS package

## Installation

### 1. Clone the repository

To get the code, clone this repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/AUROVA-LAB/aurova_preprocessed.git
```

### 2. Build the package

Navigate to your catkin workspace directory and use `catkin_make` to build the node:

```bash
cd ~/catkin_ws/pc_features
catkin_make --only-pkg-with-deps pc_feature_extraction
```

### 3. Configure the environment

After building, make sure to set up your environment with the following command:

```bash
source ~/catkin_ws/devel/setup.bash
```

## Usage

### 1. Launch the node

The node can be executed using a launch file that will initiate the point cloud transformation process. To run it, use the following command:

```bash
roslaunch pc_feature_extraction ouster_image.launch
```

Ensure that the LiDAR sensor parameters and point cloud paths are correctly configured.
