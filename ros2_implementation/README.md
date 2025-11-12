# ROS2 Visual Odometry

Monocular visual odometry for KITTI dataset using ROS2. Converted from the original C++ implementation.

## What It Does

Estimates camera trajectory from sequential images using:
- Feature detection and tracking
- Essential matrix for pose estimation  
- GPS/OXTS data for accurate scale
- Real-time visualization with ground truth comparison

## Setup

```bash
# 1. Clone repository
cd ~/ros2_ws/src
git clone https://github.com/TejaswiniDilip18/Visual-Odometry.git

# 2. Navigate to ROS2 implementation
cd Visual-Odometry/ros2_implementation

# 3. Build UTM library
cd visual_odometry
# Recompile UTM library if necessary
g++ -shared -fPIC -o libutm_converter.so utm_wrapper.cpp LatLong-UTMconversion.cpp 
cd ..

# 4. Build ROS2 package
cd ~/ros2_ws
colcon build --packages-select visual_odometry
source install/setup.bash
```

## Run
Make sure you are updating paths in launch files.

**Dataset Mode:**
```bash
ros2 launch visual_odometry vo_dataset.launch.py
```
**Camera Mode:**
```bash
ros2 launch visual_odometry vo_camera.launch.py
```
**Visualization:**
```bash
rviz2  # Set Fixed Frame to "odom", add Path and PoseStamped displays
```

## What You'll See

**RViz2 visualization:**
- Red path: Estimated trajectory
- Green path: GPS ground truth
- Blue arrow: Current pose
- Coordinate frame transforms

**ROS2 Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/vo/pose` | `PoseStamped` | Current camera pose |
| `/vo/path` | `Path` | Estimated trajectory |
| `/vo/odometry` | `Odometry` | Full odometry with covariance |
| `/vo/ground_truth/pose` | `PoseStamped` | Ground truth pose |
| `/vo/ground_truth/path` | `Path` | Ground truth trajectory |
| `/vo/current_frame` | `Image` | Current camera image |

## Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `folder_path` | Path to KITTI image folder | - |
| `oxts_data` | Path to GPS/IMU data | - |
| `true_pose` | Path to ground truth poses | - |
| `frame_rate` | Processing rate (Hz) | `10.0` |
| `min_num_features` | Min features before re-detection | `2000` |
| `use_camera` | Use live webcam | `false` |
| `camera_id` | Webcam device ID | `0` |

## Dataset Structure
```
dataset/
├── 2011_10_03_drive_0027_sync/
│   └── 2011_10_03/
│       └── 2011_10_03_drive_0027_sync/
│           ├── image_02/data/*.png
│           └── oxts/data/*.txt
└── data_odometry_poses/
    └── dataset/poses/00.txt
```


## Environment

Runs in the same Docker container as the C++ version:
- ROS2 Humble
- OpenCV 4.x
- Python 3.10+

**Platform:**
- Pre-compiled library is for Linux x86_64
- Recompile UTM library for other platforms

---

## Known Limitations

- Monocular scale drift without GPS
- Assumes forward-dominant motion (vehicle driving)
- Pure rotation not well-handled
- No loop closure

## Branch Structure

- `main` - Original C++ implementation
- `ros2_version` - This ROS2 Python implementation

---

*Personal implementation for learning visual odometry and ROS2 integration.*