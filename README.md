# Sheep Following System

This repository contains a ROS2 package for following sheep using YOLO object detection. The system provides two different approaches: high-level navigation using Nav2 and low-level velocity control.

## Package Structure

```
sheep_follower/
├── package.xml                    # Package manifest
├── setup.py                       # Python package setup
├── setup.cfg                      # Setup configuration
├── resource/                      # Package resources
│   └── sheep_follower
├── sheep_follower/                # Python package directory
│   ├── __init__.py
│   ├── nav2_sheep_follower.py     # Nav2-based follower
│   └── direct_sheep_follower.py   # Direct velocity follower
├── launch/                        # Launch files
│   ├── nav2_sheep_follower.launch.py
│   ├── direct_sheep_follower.launch.py
│   └── sheep_follower_complete.launch.py
├── config/                        # Configuration files
│   ├── nav2_sheep_follower.yaml
│   └── direct_sheep_follower.yaml
└── test/                          # Test files
    ├── test_copyright.py
    └── test_flake8.py
```

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url> yolo_sheep_follower
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select sheep_follower
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

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

# Follow any sheep
ros2 run sheep_follower nav2_sheep_follower --id sheep
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

# Follow any sheep
ros2 run sheep_follower direct_sheep_follower --id sheep
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
You can also modify the YAML configuration files in the `config/` directory:
- `nav2_sheep_follower.yaml`: Configuration for the Nav2-based follower
- `direct_sheep_follower.yaml`: Configuration for the direct velocity follower

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
- `/yolo/detections_3d` (yolo_msgs/DetectionArray): YOLO detection results

### Output Topics
- Nav2 Sheep Follower: `/navigate_to_pose` (nav2_msgs/NavigateToPose)
- Direct Velocity Follower: `/cmd_vel` (geometry_msgs/Twist)

