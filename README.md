[![CircleCI](https://circleci.com/gh/robustrobotics/flame_ros/tree/master.svg?style=shield)](https://circleci.com/gh/robustrobotics/flame_ros/tree/master)

# flame_ros
**FLaME** (Fast Lightweight Mesh Estimation) is a lightweight, CPU-only method
for dense online monocular depth estimation. Given a sequence of camera images
with known poses, **FLaME** is able to reconstruct dense 3D meshes of the
environment by posing the depth estimation problem as a variational optimization
over a Delaunay graph that can be solved at framerate, even on computationally
constrained platforms.


> :warning: **Attention please: This is a refactored version of the original FLaME [here](<https://github.com/ctu-mrs/flame/tree/ros2>)**

The `flame_ros` repository contains the ROS bindings, visualization code, and
offline frontends for the algorithm. The core library can be
found [here](<https://github.com/ctu-mrs/flame/tree/ros2>). <!--[here](https://github.com/robustrobotics/flame.git)-->

<p align="center">
    <a href="https://www.youtube.com/watch?v=vB_F-Sj0AX0">
    <img src="https://img.youtube.com/vi/vB_F-Sj0AX0/0.jpg" alt="FLaME">
    </a>
</p>

### Related Publications:
* [**FLaME: Fast Lightweight Mesh Estimation using Variational Smoothing on Delaunay Graphs**](https://groups.csail.mit.edu/rrg/papers/greene_iccv17.pdf),
*W. Nicholas Greene and Nicholas Roy*, ICCV 2017.

## Author
- W. Nicholas Greene (wng@csail.mit.edu)

## Quickstart


## Dependencies
- Ubuntu 24.04
- ROS2 Jazzy
- OpenCV 4.6.0
- Boost 1.83.0
- PCL 1.14.0
- Eigen 3.4
- Sophus (SHA: b474f05f839c0f63c281aa4e7ece03145729a2cd)
- [flame](https://github.com/ctu-mrs/flame/tree/ros2) <!--- [flame](https://github.com/robustrobotics/flame.git)"-->

## Quickstart

1. Add CTU MRS PPA:

```bash
curl https://ctu-mrs.github.io/ppa2-stable/add_ros_ppa.sh | bash
sudo apt update
```

2. Install all the packages:

```bash
sudo apt install ros-jazzy-mrs-bluefox2 ros-jazzy-mrs-mrs-serial ros-jazzy-mrs-flame ros-jazzy-mrs-flame-ros-msgs ros-jazzy-mrs-flame-ros ros-jazzy-mrs-open-vins-core ros-jazzy-mrs-rviz-plugins ros-jazzy-mrs-uav-flightforge-simulator
```

## Installation
**NOTE:** These instructions assume you are running ROS2 Jazzy on Ubuntu 24.04
and are interested in installing both `flame` and `flame_ros`. See the
installation instructions for `flame` if you only wish to install `flame`.

1. Install `apt` dependencies:
```bash
sudo apt-get install libboost-all-dev libpcl-dev python-catkin-tools
```

2. Create a Catkin workspace using `catkin_tools`:
```bash
# Source ROS.
source /opt/ros/kinetic/setup.sh

# Create workspace source folder.
mkdir -p flame_ws/src

# Checkout flame and flame_ros into workspace.
cd flame_ws/src
git clone https://github.com/robustrobotics/flame.git
git clone https://github.com/robustrobotics/flame_ros.git

# Initialize workspace.
cd ..
catkin init

# Install ROS dependencies using rosdep.
rosdep install -iy --from-paths ./src
```

3. Install Eigen 3.2 and Sophus using the scripts provided with `flame`:
```bash
# Create a dependencies folder.
mkdir -p flame_ws/dependencies/src

# Checkout Eigen and Sophus into ./dependencies/src and install into ./dependencies.
cd flame_ws
./src/flame/scripts/eigen.sh ./dependencies/src ./dependencies
./src/flame/scripts/sophus.sh ./dependencies/src ./dependencies

# Copy and source environment variable script:
cp ./src/flame/scripts/env.sh ./dependencies/
source ./dependencies/env.sh
```

4. Build workspace:
```bash
# Build!
catkin build

# Source workspace.
source ./devel/setup.sh
```

## Offline Processing
Two types of offline nodes are provided, one to process video from
the
[EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and
one to process video from
the
[TUM RGB-D SLAM Benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset).

### EuRoC Data
First, download and extract one of the ASL
datasets
[here](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) (the
Vicon Room datasets should work well).

Next, update the parameter file in `flame_ros/cfg/flame_offline_asl.yaml` to
point to where you extracted the data:
```bash
pose_path: <path_to_dataset>/mav0/state_groundtruth_estimate0
rgb_path: <path_to_dataset>/mav0/cam0
```

Finally, to process the data launch:
```bash
roslaunch flame_ros flame_offline_asl.launch
```
The mesh should be published on the `/flame/mesh` topic. To visualize this topic
in rviz, consult the the [Visualization](#Visualization) section.

### TUM Data
First, download and extract one of the
datasets [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/download)
(`fr3/structure_texture_far` or `fr3/long_office_household` should work well).
Use the `associate.py`
script [here](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools) to
associate the pose (`groundtruth.txt`) and RGB (`rgb.txt`) files (you can
associate the depthmaps as well).

A ROS-compliant camera calibration YAML file will be needed. You can use the one
provided in `flame_ros/cfg/kinect.yaml`, which has the default parameters for
the Microsoft Kinect used to collect the TUM datasets.

Next, update the parameter file in `flame_ros/cfg/flame_offline_tum.yaml` to
point to where you extracted the data:
```bash
input_file: <path_to_dataset>/groundtruth_rgb.txt
calib_file: <path_to_flame_ros>/cfg/kinect.yaml
```

Finally, to process the data launch:
```bash
roslaunch flame_ros flame_offline_tum.launch
```
The mesh should be published on the `/flame/mesh` topic. To visualize this topic
in rviz, consult the the [Visualization](#Visualization) section.

## Online Processing
The online nodelet can be launched using `flame_nodelet.launch`:
```bash
roslaunch flame_ros flame_nodelet.launch image:=/image
```
where `/image` is your live rectified/undistorted image stream. The `frame_id` of
this topic must correspond to a Right-Down-Forward frame attached to the
camera. The `tf` tree must be complete such that the pose of the camera in the
world frame can be resolved by `tf`.

The mesh should be published on the `/flame/mesh` topic. To visualize this topic
in rviz, consult the the [Visualization](#Visualization) section.

The `flame_nodelet.launch` launch file loads the parameters listed in
`flame_ros/cfg/flame_nodelet.yaml`. You may need to update the
`input/camera_frame_id` param for your data. See the [Parameters](#Parameters)
section for more parameter information.

## Visualization
You can use the provided configuration file (`flame_ros/cfg/flame.rviz`) to
visualize the output data in rviz. This approach uses a custom rviz plugin to
render the `/flame/mesh` messages. `flame_ros` can also publish the depth data
in other formats (e.g. as a `sensor_msgs/PointCloud2` or a `sensor_msgs/Image`),
which can be visualized by enabling the corresponding plugins.

## Parameters
There are many parameters that control processing, but only a handful are
particularly important:

- `output/*`: Controls what type of output messages are produced. If you are
  concerned about speed, you should prefer publishing only the mesh data
  (`output/mesh: True`), but other types of output can be enabled here.

- `debug/*`: Controls what type of debug images are published. Creating these
  images is relatively expensive, so disable for real-time operation.

- `threading/openmp/*`: Controls OpenMP-accelerated sections. You may wish to
  tune the number of threads per parallel section
  (`threading/openmp/num_threads`) or the number of chunks per thread
  (`threading/openmp/chunk_size`) for your processor.

- `features/detection/min_grad_mag`: Controls the minimum gradient magnitude for
  detected features.

- `features/detection/win_size`: Features are detected by dividing the image
  domain into `win_size x win_size` blocks and selecting the best trackable
  pixel inside each block. Set to a large number (e.g. 32) for coarse, but fast
  reconstructions, and a small number (e.g. 8) for finer reconstructions.

- `regularization/nltgv2/data_factor`: Controls the balance between smoothing
  and data-fitting in the regularizer. It should be set in relation to the
  detection window size. Some good values are 0.1-0.25.

# Complete parameter list reference

Mapping between ROS configuration parameters and algorithm parameters from the paper "FLaME: Fast Lightweight Mesh Estimation using Variational Smoothing on Delaunay Graphs" (Greene & Roy, ICCV 2017).

## Feature Detection & Tracking

### `features.detection.win_size` → Detail level L (Section 3.1)
- **Units:** pixels
- **Description:** Grid cell size is 2^L × 2^L pixels for feature sampling
- **Example:** win_size=16 corresponds to L≈4, giving 16×16 pixel cells
- **Effect:** Smaller L = denser features (higher quality, slower); Larger L = sparser features (faster, lower quality)

### `features.detection.min_grad_mag` → Gradient threshold
- **Units:** intensity (0-255 for grayscale)
- **Config value:** 5.0
- **Description:** Minimum image gradient magnitude for feature detection
- **Typical range:** 3-15
- **Effect:** Higher values = fewer but more trackable features

### `features.tracking.win_size` → Patch matching window
- **Units:** pixels
- **Config value:** 5 (creates 5×5 pixel patch)
- **Description:** Size of image patch used for epipolar line matching
- **Typical range:** 3-9 pixels (must be odd)
- **Effect:** Larger window = more robust but slower matching

### `features.tracking.epipolar_line_var` → σ²_z (Equation 7)
- **Units:** (1/meters)²
- **Config value:** 10.0
- **Description:** Noise variance for inverse depth measurements from stereo matching
- **Effect:** Higher values = less trust in individual measurements, more reliance on smoothing
- **Note:** High value (10.0) indicates very uncertain stereo matching, favoring aggressive smoothing

## Graph Construction

### `regularization.nltgv2.idepth_var_max` → σ²_max (Section 3.2)
- **Units:** (1/meters)²
- **Config value:** 0.01
- **Description:** Maximum inverse depth variance before feature is added to mesh
- **Effect:** Filters out uncertain depth estimates from graph optimization
- **Example:** 0.01 accepts features with depth uncertainty ~±10cm at 1m distance

## Optimization Parameters (Chambolle-Pock Algorithm)

### `regularization.nltgv2.data_factor` → λ (Equation 15)
- **Units:** dimensionless
- **Config value:** 0.15
- **Description:** Balance between data fidelity and smoothness in NLTGV²-L1 cost
- **Typical range:** 0.1-0.35 (paper recommendation)
- **Effect:** Higher = trust raw measurements more; Lower = smooth more aggressively

### `regularization.nltgv2.step_x` → τ (Algorithm 2)
- **Units:** dimensionless
- **Config value:** 0.001
- **Description:** Primal step size - how much vertex depths change per iteration
- **Effect:** Too large = unstable; Too small = slow convergence
- **Note:** Conservative value prioritizes stability over speed

### `regularization.nltgv2.step_q` → σ (Algorithm 2)
- **Units:** dimensionless
- **Config value:** 125.0
- **Description:** Dual step size - how aggressively smoothness constraint penalties update
- **Constraint:** Must satisfy στ < 1 (here: 125.0 × 0.001 = 0.125 ✓)
- **Note:** Dual steps can be much larger than primal steps

### `regularization.nltgv2.theta` → θ (Algorithm 2)
- **Units:** dimensionless
- **Config value:** 0.25
- **Description:** Extrapolation/momentum parameter for accelerated convergence
- **Range:** [0, 1] where 0 = no acceleration, 1 = maximum acceleration
- **Effect:** Higher = faster convergence but less stable

## Edge Weights (hardcoded in implementation)

### `e_α = 1/||v_i_u - v_j_u||²` → α weights (Equation 11)
- **Units:** 1/pixels²
- **Description:** First-order smoothness weights, inversely proportional to edge length
- **Example:** 5-pixel edge → weight = 0.04; 50-pixel edge → weight = 0.0004
- **Effect:** Shorter edges have stronger influence on planar smoothing

### `e_β = 1` → β weights (Equation 12)
- **Units:** dimensionless
- **Description:** Second-order smoothness weights (constant across all edges)
- **Effect:** Controls smoothing of auxiliary variable w (local plane slopes)

## Processing Control

### `input.subsample_factor`
- **Units:** frames
- **Config value:** 1
- **Description:** Process every Nth input frame
- **Effect:** Higher values reduce CPU load but decrease temporal resolution

### `input.poseframe_subsample_factor`
- **Units:** frames
- **Config value:** 6
- **Description:** Update pose graph every Nth frame
- **Effect:** At 30fps with value 6 → updates every 200ms

## Output Filtering

### `output.min_triangle_idepth`
- **Units:** 1/meters
- **Config value:** 0.01 (corresponds to 100m maximum depth)
- **Description:** Minimum inverse depth for triangle display
- **Effect:** Filters distant/uncertain triangles from visualization

### `output.edge_length_thresh`
- **Units:** fraction of image width
- **Config value:** 0.333
- **Description:** Maximum edge length for display
- **Example:** 0.333 × 640px = 213 pixels maximum edge length
- **Effect:** Prevents long, unreliable triangles in output mesh

### `output.oblique_normal_thresh`
- **Units:** radians
- **Config value:** 1.57 (~90 degrees)
- **Description:** Maximum angle between triangle normal and viewing direction
- **Effect:** Filters out oblique (edge-on) triangles from display

## Tuning Guidelines

**Unstable optimization?**
- Reduce `step_x` and `step_q`
- Ensure στ < 1 constraint is satisfied

**Too slow convergence?**
- Increase `theta` (but keep < 1)
- Increase `step_x` and `step_q` carefully

**Noisy depth estimates?**
- Increase `epipolar_line_var`
- Decrease `data_factor` (trust smoothing more)

**Over-smoothed results?**
- Decrease `epipolar_line_var`
- Increase `data_factor` (trust measurements more)
- Use smaller detail level L (denser features)

**Want faster processing?**
- Increase detail level L (sparser features)
- Increase `subsample_factor`
- Increase `idepth_var_max` (accept less certain features)

## Algorithm Overview

FLaME reformulates dense monocular reconstruction as a graph-based variational optimization problem:

1. **Features** are detected and tracked along epipolar lines
2. **Delaunay triangulation** creates a mesh connecting converged features
3. **NLTGV² regularization** promotes piecewise-planar structure via graph optimization
4. **Primal-dual optimization** alternates between updating vertex depths and edge constraints

The key insight: mesh edges enforce second-order smoothness constraints that adapt to local geometry, producing planar surfaces with sharp boundaries rather than uniformly smooth reconstructions.

### Performance Tips
For best results use a high framerate (>= 30 Hz) camera with VGA-sized
images. Higher resolution images will require more accurate poses. The feature
detection window size (`features/detection/win_size`) and the data scaling term
(`regularization/nltgv2/data_factor`) are the primary knobs for tuning
performance. The default parameters should work well in most cases, but you may
need to tune for your specific data.

By default, `flame_ros` will publish several debug images. While helpful to
observe during operation, they will slow down the pipeline. Disable them if you
are trying to increase throughput.

The usual tips for monocular SLAM/depth estimation systems also apply:
- Prefer slow translational motion
- Avoid fast rotations when possible
- Use an accurate pose source (e.g. one of the many available visual
  odometry/SLAM packages)
- Prefer texture-rich environments
- Prefer environments with even lighting
