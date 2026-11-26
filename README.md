# Mechabot — ROS 2 Mobile Robot

Mechabot is a custom-built indoor mobile robot developed with ROS 2 Humble. The project brings together a physical differential-drive robot, simulation, LiDAR mapping, localization, and autonomous navigation.

## Project gallery

Photos and demonstration videos will be added here.

## What Mechabot can do

- **Indoor mapping:** Build a map using LiDAR and SLAM Toolbox.
- **Localization:** Estimate the robot's position on a saved map using AMCL.
- **Autonomous navigation:** Navigate to a goal using Nav2. Mechabot has successfully navigated to a goal in a room on the real robot.
- **Simulation:** Model and test Mechabot in Gazebo and visualize its state in RViz.
- **Manual control:** Control the robot using teleoperation.

## Hardware

- Raspberry Pi 4 and ESP32
- Differential-drive geared motors with encoders
- RPLIDAR A1 LiDAR, IMU, and camera
- Custom chassis and 3D-printed components

## Software

ROS 2 Humble · Gazebo · RViz2 · ros2_control · SLAM Toolbox · AMCL · Nav2

## Repository packages

| Package | Purpose |
| --- | --- |
| `mechabot_bringup` | Robot and simulation launch files |
| `mechabot_controller` | Controller and teleoperation configuration |
| `mechabot_description` | Robot description, meshes, simulation worlds, and visualization |
| `mechabot_mapping` | SLAM configuration and maps |
| `mechabot_localization` | Localization configuration |
| `mechabot_navigation` | Nav2 configuration and launch files |
| `mechabot_scripts` | Sensor and experimental robot scripts |

## Experiments

- **Dijkstra planning:** A custom planner has been developed to generate a path and visualize visited cells.
- **Camera-based line following:** Tested in Gazebo; sharp corners and higher speeds need further tuning.

The experiment files may be in a local workspace and are not necessarily present on this branch.

## Setup and operation

Detailed build instructions, hardware setup, and verified simulation and real-robot launch commands will be added after checking the latest workspace.
