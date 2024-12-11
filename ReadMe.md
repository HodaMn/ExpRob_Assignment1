# Experimental Robotics Project

This repository includes ROS packages and nodes developed for a robotics integration project. The goal is to combine a robotic arm with a camera system to detect markers, map the environment, and execute movements using MoveIt. The project showcases setting up a simulation, recognizing visual markers, and controlling the robotic arm to approach and align with these markers.

## Overview

The project consists of:

- **Simulation Environment:**
  - Gazebo setup, several marker objects placed in different locations, and a robotic arm.

- **ROS Nodes:**
  - **Robot Model Loader:** Loads the robot's URDF and configures MoveIt for motion planning.
  - **Arm Controller:** Plans and executes the robotic arm's movements.
  - **Camera Processor:** Captures and processes images to detect and track markers.
  - **Camera Rotator:** Adjusts the camera or arm position to align with detected markers.

## Key Features

- **Marker Detection:**
  - Identifies marker IDs and their positions from camera images.
  
- **Dynamic Alignment:**
  - Rotates the camera or moves the arm to align with each detected marker.
  
- **Motion Planning:**
  - Utilizes MoveIt to compute and execute paths for the robotic arm to reach target poses.
  
- **Inverse Kinematics:**
  - Calculates joint configurations to achieve specific end-effector positions and orientations.

## Components

  **Marker Detection and Camera Control:**
   - Subscribes to camera image topics.
   - Uses OpenCV to detect markers and determine their 2D coordinates.
   - Records marker IDs and locations.
   - Adjusts camera or arm orientation to focus on each marker.
   - Highlights detected markers in the camera feed.

  **Service Interfaces:**
   - Offers ROS services to trigger arm movements to specific poses.

## Getting Started
### Prerequisites

- **ROS:** Compatible with ROS Noetic or Melodic.
- **MoveIt:** For motion planning and arm control.
- **Gazebo:** Simulation environment.
- **OpenCV:** For image processing.
- **Python 3:** Along with necessary Python libraries.

### Python dependencies installation 
```
pip install numpy scipy imutils opencv-python
```
