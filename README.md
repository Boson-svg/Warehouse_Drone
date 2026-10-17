# Warehouse_Drone

## Overview

`warehouse_ws` is a ROS 2 Humble workspace for an intelligent warehouse drone system running on a Sunrise-based onboard computer.

The project focuses on autonomous flight in warehouse environments, including flight-controller communication, LiDAR-based localization, mission execution, onboard perception, and system orchestration.

## Typical Use Cases

- Autonomous inspection in warehouse environments
- Mission-based takeoff, route execution, and action control
- Stable flight with LiDAR-assisted localization and mapping
- Onboard visual tasks such as image streaming, target recognition, and barcode-related processing

## Key Features

- Organized as a ROS 2 multi-package workspace
- Intended for deployment on a Sunrise onboard computer
- Uses `mavros` for flight-controller communication
- Uses Livox MID360 and FastLIO in the localization pipeline
- Supports mission configuration, flight control, status coordination, and perception input

## Core System Flow

A typical runtime chain is:

1. `mavros` connects to the flight controller
2. `livox_ros_driver2` provides MID360 LiDAR data
3. `fast_lio` outputs odometry/localization
4. `drone_localization` bridges localization data into MAVROS
5. `drone_control` provides TF and mission execution logic
6. `drone_perception` handles onboard perception input
7. `drone_bringup` manages startup orchestration and supervision

## Workspace Packages

### 1. drone_bringup
Handles startup orchestration, node launching, readiness checks, and supervisor logic.

Main responsibilities:
- Launch core nodes in sequence
- Wait for required topics to become ready
- Manage the onboard startup flow
- Support automatic startup with system service integration

### 2. drone_control
Handles mission control, action execution, and flight-control-related bridge logic.

Main responsibilities:
- Provide TF for control logic
- Execute mission actions and flight procedures
- Connect mission configuration to runtime execution
- Support the offboard control workflow

### 3. drone_localization
Handles localization bridging and pose consistency monitoring.

Main responsibilities:
- Convert and bridge FastLIO outputs into MAVROS
- Feed external/vision-based pose data to the flight controller
- Monitor deviation between localization pose and flight-controller local pose
- Support abnormal-condition detection and restart decisions

### 4. drone_mission
Stores and manages mission configuration.

Main responsibilities:
- Keep mission YAML files
- Describe waypoints, actions, and process parameters
- Serve as the configuration input for task execution

### 5. drone_msgs
Defines shared message and service interfaces used across the workspace.

Main responsibilities:
- Provide common msg/srv definitions
- Standardize task, status, and perception-related interfaces
- Reduce coupling between packages

### 6. drone_perception
Handles onboard perception input and result processing.

Main responsibilities:
- Receive TCP/JPEG data from the K230 camera side
- Process image and perception results
- Support target recognition, barcode-related capabilities, and image-link integration
- Provide perception input for warehouse-oriented tasks

### 7. drone_qt_2
Acts as an onboard-side or relay-side support program.

Main responsibilities:
- Store or forward task-related data
- Bridge data between the ground side and the onboard side
- Assist with mission triggering and perception-result return paths

### 8. drone_warehouse
Reserved or domain-specific package for warehouse-related functionality.

Based on the current repository state, this package exists in the workspace but has limited public description. From its name and project context, it appears to be intended for intelligent warehouse scenario extensions.

## External Dependencies

Important external dependencies include:

- `mavros`
- `livox_ros_driver2`
- `fast_lio`

Depending on the deployment, the system may also rely on:

- ROS 2 Humble
- Livox MID360
- A flight controller with MAVLink connectivity
- Onboard vision hardware such as K230

## Deployment Notes

This repository is suitable as:

- A ROS 2 workspace source tree
- A development and analysis workspace for the onboard system
- A software integration base for an intelligent warehouse drone project

The actual runtime environment is typically a Linux-based onboard device, while the current Windows copy can be used for reading, organizing, analyzing, and editing the code.

## Intended Audience

This project is useful for work involving:

- Intelligent warehouse drone development
- ROS 2 multi-package integration
- LiDAR localization and flight-controller integration
- Joint development of onboard perception and mission systems
