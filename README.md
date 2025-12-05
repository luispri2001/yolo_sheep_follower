# Sheep Following System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/luispri2001/yolo_sheep_follower)](https://github.com/luispri2001/yolo_sheep_follower/stargazers)

This repository contains a ROS2 package for following sheep using YOLO object detection. The system provides two different approaches: high-level navigation using Nav2 and low-level velocity control.

## Demos
<div align="center">
  <a href="https://drive.google.com/file/d/1BushD_RQRCJy4ibuARQTfjgNHNZmI35j/view?usp=sharing" target="_blank">
    <img src="https://img.shields.io/badge/▶️-Simulation_Demo-red?style=for-the-badge" alt="Video Demo" width="300"/>
  </a>
</div>

<div align="center">
  <a href="https://drive.google.com/file/d/1lewlBhPj2SrJkG1iUKibFtqL6TAkV6qo/view?usp=sharing" target="_blank">
    <img src="https://img.shields.io/badge/▶️-Real_World_Demo-red?style=for-the-badge" alt="Video Demo" width="300"/>
  </a>
</div>

## Repository Structure

```
yolo_sheep_follower/
├── sheep_follower/                    # Main ROS2 package
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   ├── sheep_follower/                # Python source code
│   │   ├── __init__.py
│   │   ├── nav2_sheep_follower.py
│   │   ├── direct_sheep_follower.py
│   │   └── wolf_distance_detector.py
│   ├── launch/
│   │   ├── sheep_follower_simulation.launch.py
│   │   ├── nav2_sheep_follower.launch.py
│   │   └── direct_sheep_follower.launch.py
│   └── config/
│       ├── nav2_sheep_follower.yaml
│       ├── direct_sheep_follower.yaml
│       └── sheep_follower.rviz
├── deps/                              # Git submodules
│   ├── yolo_ros/                      # YOLO ROS2 (includes yolo_msgs)
│   └── gps_ignition_simulation/       # Gazebo worlds & robot models
├── logos/
├── .gitignore
├── .gitmodules
├── CITATION.cff
└── README.md
```

## Installation

```bash
# Clone with submodules
cd ~/ros2_ws/src
git clone --recursive https://github.com/luispri2001/yolo_sheep_follower.git

# Install ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source
source install/setup.bash
```

> **Note:** If you cloned without `--recursive`, run: `git submodule update --init --recursive`

## Quick Start - Complete Simulation

Launch the full sheep following simulation with one command:

```bash
ros2 launch sheep_follower sheep_follower_simulation.launch.py
```

This launches:
- Gazebo Ignition with campus world and sheep
- LeoRover robot
- Nav2 navigation stack
- YOLO 3D detection
- Sheep follower node

### Simulation Parameters

```bash
ros2 launch sheep_follower sheep_follower_simulation.launch.py \
  sheep_id:=any \
  yolo_model:=yolov8m.pt \
  yolo_device:=cuda:0 \
  use_rviz:=true
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sheep_id` | `any` | Target sheep ID or "any" to follow first detected |
| `yolo_model` | `yolov8m.pt` | YOLO model to use |
| `yolo_device` | `cuda:0` | Device for inference (cuda:0, cpu) |
| `use_rviz` | `true` | Launch RViz with visualization |
| `use_sim_time` | `true` | Use simulation time |

> **Note:** The system accepts both "sheep" and "cow" detections since YOLO may confuse them in simulation.

## Requirements

- **ROS2**: Humble / Iron
- **Python**: 3.8+
- **OS**: Ubuntu 22.04

### Dependencies (auto-installed via rosdep)
- `nav2_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_geometry_msgs`
- `vision_msgs`, `std_msgs`, `visualization_msgs`, `sensor_msgs`
- `cv_bridge`, `numpy`, `opencv-python`
- `yolo_msgs` *(included as submodule)*

## Overview

This system implements autonomous sheep following capabilities for robotic applications in agricultural and research environments. It uses YOLO-based 3D object detection to identify and track specific sheep by ID or any sheep in the field.

## Components

### 1. Nav2 Sheep Follower (`nav2_sheep_follower.py`)

A high-level navigation approach that uses the Nav2 navigation stack to follow sheep. This implementation is ideal for complex environments requiring path planning and obstacle avoidance.

#### Features
- **High-level navigation**: Uses Nav2's `NavigateToPose` action for robust path planning
- **Transform handling**: Properly handles coordinate transformations between frames
- **Goal optimization**: Avoids sending redundant navigation goals
- **Graceful recovery**: Continues to last goal when sheep detection is lost
- **Configurable parameters**: Adjustable timeouts, tolerances, and target frames

#### Parameters
- `target_frame` (default: "map"): Target coordinate frame for navigation
- `lost_timeout` (default: 2.0s): Time before considering sheep lost
- `goal_tolerance` (default: 0.5m): Minimum distance between goals to send new command
- `goal_offset` (default: 1.0m): Distance to maintain from sheep position

#### Usage
```bash
# Run with launch file (recommended)
ros2 launch sheep_follower nav2_sheep_follower.launch.py sheep_id:=3

# Or run the node directly
ros2 run sheep_follower nav2_sheep_follower --id 3

# Follow any sheep (default - follows first detected and sticks with it)
ros2 run sheep_follower nav2_sheep_follower --id any
```

#### Architecture
```
YOLO Detections → TF Transformations → Goal Calculation → Nav2 Action Client
```

### 2. Direct Velocity Sheep Follower (`direct_sheep_follower.py`)

A low-level approach that directly controls robot velocity using `cmd_vel` commands. This implementation provides more immediate response and fine-grained control.

#### Features
- **Direct velocity control**: Publishes `Twist` messages to `/cmd_vel`
- **Distance-based behavior**: Adjusts speed based on proximity to target
- **Visual servoing**: Uses both 3D position and 2D image coordinates for control
- **Safety mechanisms**: Automatic stopping when target is lost
- **Real-time control**: High-frequency control loop (10Hz)

#### Parameters
- `max_linear_speed` (default: 0.3 m/s): Maximum forward/backward speed
- `max_angular_speed` (default: 0.6 rad/s): Maximum rotation speed
- `min_follow_distance` (default: 2.5m): Minimum distance to maintain
- `max_follow_distance` (default: 5.0m): Maximum following distance
- `center_threshold` (default: 30 pixels): Tolerance for centering target
- `lost_object_timeout` (default: 2.0s): Time before stopping when target lost

#### Usage
```bash
# Run with launch file (recommended)
ros2 launch sheep_follower direct_sheep_follower.launch.py sheep_id:=3

# Or run the node directly
ros2 run sheep_follower direct_sheep_follower --id 3

# Follow any sheep (default - follows first detected and sticks with it)
ros2 run sheep_follower direct_sheep_follower --id any
```

#### Control Logic
- **Linear velocity**: Based on distance to target (approach/retreat/maintain)
- **Angular velocity**: Combination of 3D lateral offset and 2D image centering
- **Safety**: Automatic stop when target lost or invalid data received

## Configuration

### Launch Parameters
You can customize the behavior of the nodes with launch file parameters:

```bash
# For Nav2 follower
ros2 launch sheep_follower nav2_sheep_follower.launch.py \
  sheep_id:=3 \
  target_frame:=map \
  lost_timeout:=3.0 \
  goal_tolerance:=0.7 \
  use_sim_time:=true

# For Direct velocity follower
ros2 launch sheep_follower direct_sheep_follower.launch.py \
  sheep_id:=3 \
  max_linear_speed:=0.4 \
  min_follow_distance:=2.0 \
  use_sim_time:=true
```

### Configuration Files
YAML configuration files in `config/`:
- `nav2_sheep_follower.yaml`
- `direct_sheep_follower.yaml`

## Dependencies

### ROS2 Packages
- `rclpy`: ROS2 Python client library
- `geometry_msgs`: For Twist and PoseStamped messages
- `nav2_msgs`: For NavigateToPose action
- `yolo_msgs`: Custom YOLO detection messages
- `tf2_ros`: Transform library
- `tf2_geometry_msgs`: Geometry message transformations
- `vision_msgs`: Vision-related message definitions

### Input Topics
- `/detections_3d` (yolo_msgs/DetectionArray): 3D YOLO detection results
- `/tracking` (yolo_msgs/DetectionArray): YOLO tracking results with IDs

### Output Topics
- Nav2 Sheep Follower: `/navigate_to_pose` (nav2_msgs/NavigateToPose)
- Direct Velocity Follower: `/cmd_vel` (geometry_msgs/Twist)

---

### Wolf Distance Detector (`wolf_distance_detector.py`)

This node detects wolves using YOLO detections and publishes both the estimated 3D distance to the wolf and a visualization. It also publishes an annotated image with the 2D bounding box and distance overlay.

#### Features

- **Wolf detection**: Identifies wolves in YOLO detection messages
- **Distance estimation**: Calculates and publishes the 3D distance to the detected wolf
- **RViz visualization**: Publishes 3D bounding box and distance as markers
- **Annotated image**: Publishes a camera image with the wolf's 2D bounding box and distance overlay

#### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/wolf/distance` | `std_msgs/Float32` | Distance to the detected wolf |
| `/wolf/visualization` | `visualization_msgs/MarkerArray` | Markers for RViz |
| `/wolf/image_detection` | `sensor_msgs/Image` | Annotated image with bounding box |

#### Usage
```bash
ros2 run sheep_follower wolf_distance_detector
```

#### Visualization
To visualize the wolf detection and distance in RViz:
1. Run the `wolf_distance_detector` node.
2. In RViz, add a "Marker" display type and subscribe to the `/wolf/visualization` topic.
3. Add a "Image" display type and subscribe to the `/wolf/image_detection` topic for annotated images.

## Citation

If you use this software, please cite the following paper:

```bibtex
@inproceedings{herds2025,
  title     = {HERDS: A ROS 2-based animal detection and herding system},
  author    = {Prieto-L{\'o}pez, Luis and Mayoko Biong, Jean Chrysostome and 
               S{\'a}nchez de la Fuente, Sergio and Gonz{\'a}lez-Santamarta, Miguel A. and 
               Rodr{\'i}guez-Lera, Francisco J. and S{\'a}nchez-Gonz{\'a}lez, Lidia},
  booktitle = {Proceedings of the Eight Iberian Robotics Conference (ROBOT 2025)},
  year      = {2025},
  month     = {November},
  publisher = {IEEE Press},
  address   = {Porto, Portugal}
}
```

## Acknowledgments

This project has been partially funded by the Recovery, Transformation, and Resilience Plan, financed by the European Union (Next Generation), thanks to the AURORAS project, and by grant PID2024-161761OB-C21 funded by MICIU/AEI/10.13039/501100011033 and by the ERDF/EU.

<p align="center">
  <img src="https://project-auroras.github.io/auroras/logos/tituloblanco.png" alt="AURORAS" width="400"/>
  <br/>
  <img src="logos/MICIUCofinanciadoAEI.jpg" alt="Funding" width="400"/>
</p>