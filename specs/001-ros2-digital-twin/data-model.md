# Data Model: Digital Twin Simulation Module

This document outlines the key entities and their relationships within the Digital Twin Simulation Module, derived from the feature specification.

## Core Entities

### 1. Digital Twin
- **Description**: A comprehensive virtual replica of a physical humanoid robot and its operational environment.
- **Attributes**:
    - `robot_model`: Reference to the `Humanoid Robot Model`.
    - `gazebo_environment`: Reference to the `Simulation Environment (Gazebo)`.
    - `unity_environment`: Reference to the `Simulation Environment (Unity)`.
    - `ros2_interface`: Reference to the `ROS 2` middleware for communication.
- **Relationships**: Composed of `Humanoid Robot Model`, `Simulation Environment (Gazebo)`, `Simulation Environment (Unity)`, and interfaces via `ROS 2`.

### 2. Humanoid Robot Model
- **Description**: A detailed simulated representation of a humanoid robot.
- **Attributes**:
    - `kinematic_structure`: Defines joints, links, and their relationships (e.g., URDF representation).
    - `dynamic_properties`: Mass, inertia, friction coefficients.
    - `visual_appearance`: Textures, materials, mesh data.
    - `sensors`: Collection of simulated `LiDAR Sensor`, `Depth Camera Sensor`, `IMU Sensor`.
    - `actuators`: Joint controllers for movement.
- **Relationships**: Part of a `Digital Twin`. Equipped with various `Sensor` types.

### 3. Simulation Environment (Gazebo)
- **Description**: A 3D physics simulator for modeling robot dynamics, collisions, and sensor interactions.
- **Attributes**:
    - `physics_engine_settings`: Gravity, simulation speed, solver parameters.
    - `world_definition`: Static obstacles, terrains, lighting.
    - `robot_instances`: Deployed `Humanoid Robot Model` instances.
- **Relationships**: Interacts with `Humanoid Robot Model` and utilizes `Physics Engine`. Communicates via `ROS 2`.

### 4. Simulation Environment (Unity)
- **Description**: A real-time 3D development platform for creating visually rich, high-fidelity environments and advanced human-robot interaction scenarios.
- **Attributes**:
    - `visual_assets`: 3D models, textures, lighting, post-processing effects.
    - `interaction_logic`: Scripts for human-robot interaction, object manipulation.
    - `ros2_interface_config`: Configuration for ROS 2 communication (e.g., ROS-TCP-Endpoint).
- **Relationships**: Visualizes `Humanoid Robot Model`. Enables `Human-Robot Interaction`. Communicates via `ROS 2`.

### 5. ROS 2 (Robot Operating System 2)
- **Description**: Middleware for communication and coordination between different components of the digital twin.
- **Attributes**:
    - `topics`: Data streams (e.g., sensor data, joint states, command velocities).
    - `services`: Request/response communication (e.g., robot commands, environment queries).
    - `actions`: Long-running tasks (e.g., navigation goals).
- **Relationships**: Connects `Humanoid Robot Model`, `Simulation Environment (Gazebo)`, `Simulation Environment (Unity)`, and external control systems.

### 6. Physics Engine
- **Description**: Software component responsible for calculating physical behaviors.
- **Attributes**: Not directly exposed as a separate entity in the data model but a core component of `Simulation Environment (Gazebo)` and `Simulation Environment (Unity)`.
- **Relationships**: Integral to `Simulation Environment (Gazebo)` and `Simulation Environment (Unity)`.

### 7. LiDAR Sensor
- **Description**: A simulated laser-based sensor providing precise distance measurements.
- **Attributes**:
    - `scan_pattern`: Horizontal/vertical resolution, angular range.
    - `max_range`, `min_range`: Detection limits.
    - `noise_model`: Simulation of sensor noise.
    - `ros2_topic`: Topic for publishing `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2`.
- **Relationships**: Attached to `Humanoid Robot Model`. Generates data consumed via `ROS 2`.

### 8. Depth Camera Sensor
- **Description**: A simulated camera providing depth information.
- **Attributes**:
    - `image_resolution`: Width, height.
    - `field_of_view`: Horizontal, vertical.
    - `min_depth`, `max_depth`: Depth detection limits.
    - `ros2_topic`: Topic for publishing `sensor_msgs/msg/Image` (depth image) and optionally `sensor_msgs/msg/PointCloud2`.
- **Relationships**: Attached to `Humanoid Robot Model`. Generates data consumed via `ROS 2`.

### 9. IMU Sensor
- **Description**: A simulated Inertial Measurement Unit providing data on orientation, angular velocity, and linear acceleration.
- **Attributes**:
    - `update_rate`: Frequency of data publication.
    - `noise_model`: Simulation of sensor noise and drift.
    - `ros2_topic`: Topic for publishing `sensor_msgs/msg/Imu`.
- **Relationships**: Attached to `Humanoid Robot Model`. Generates data consumed via `ROS 2`.

## Relationships Summary

- A `Digital Twin` is composed of a `Humanoid Robot Model`, `Gazebo` and `Unity` environments, all interconnected by `ROS 2`.
- A `Humanoid Robot Model` is equipped with `LiDAR`, `Depth Camera`, and `IMU` `Sensors`.
- `Gazebo` and `Unity` environments rely on a `Physics Engine` for realistic simulations.
- `ROS 2` facilitates all inter-component communication within the digital twin setup.