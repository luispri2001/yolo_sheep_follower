# Sheep Following System

This repository contains two ROS2 nodes for following sheep using YOLO object detection. The system provides two different approaches: high-level navigation using Nav2 and low-level velocity control.

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
# Follow sheep with ID "3"
ros2 run sheep_follower nav2_sheep_follower.py --id 3

# Follow any sheep
ros2 run sheep_follower nav2_sheep_follower.py --id sheep
```

#### Architecture
```
YOLO Detections → TF Transformations → Goal Calculation → Nav2 Action Client
```

### 2. Direct Velocity Sheep Follower (`sheep_follower.py`)

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
