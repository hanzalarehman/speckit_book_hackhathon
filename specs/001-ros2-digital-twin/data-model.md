# Data Model for Digital Twin Simulation Module

This document outlines the key entities and their relationships within the Digital Twin Simulation for Humanoid Robots module.

## Entities

### Digital Twin
- **Description**: A comprehensive virtual replica of a physical humanoid robot and its operational environment, enabling simulation, monitoring, and control.
- **Attributes**:
    - `id`: Unique identifier (string)
    - `name`: Human-readable name (string)
    - `robot_model_id`: Reference to Humanoid Robot Model (string)
    - `gazebo_environment_id`: Reference to Gazebo Simulation Environment (string, optional)
    - `unity_environment_id`: Reference to Unity Simulation Environment (string, optional)
    - `ros2_config`: Configuration details for ROS 2 integration (JSON/YAML)
    - `status`: Current operational status (e.g., "running", "paused", "stopped")

### Humanoid Robot Model
- **Description**: A detailed simulated representation of a humanoid robot, including its kinematic structure, dynamic properties (mass, inertia), and visual appearance.
- **Attributes**:
    - `id`: Unique identifier (string)
    - `name`: Robot model name (string)
    - `urdf_description`: Path or content of URDF/SDF file (string)
    - `mass_properties`: Details on mass, inertia (JSON/YAML)
    - `kinematic_structure`: Joint and link hierarchy (JSON/YAML)
    - `sensor_ids`: List of associated sensor IDs (array of strings)

### Simulation Environment (Gazebo)
- **Description**: A powerful 3D physics simulator used for accurately modeling robot dynamics, collisions, and sensor interactions.
- **Attributes**:
    - `id`: Unique identifier (string)
    - `name`: Environment name (string)
    - `world_file`: Path or content of Gazebo world file (string)
    - `physics_engine_config`: Configuration for the physics engine (JSON/YAML)
    - `robot_model_ids`: List of robot models present in this environment (array of strings)

### Simulation Environment (Unity)
- **Description**: A real-time 3D development platform used for creating visually rich, high-fidelity environments and advanced human-robot interaction scenarios.
- **Attributes**:
    - `id`: Unique identifier (string)
    - `name`: Environment name (string)
    - `scene_file`: Path or content of Unity scene file (string)
    - `graphics_config`: Configuration for rendering and visual quality (JSON/YAML)
    - `ros2_bridge_config`: Configuration for ROS 2 Unity communication (JSON/YAML)
    - `robot_model_ids`: List of robot models present in this environment (array of strings)

### ROS 2
- **Description**: The Robot Operating System 2, serving as the middleware for communication and coordination between different components of the digital twin.
- **Attributes**:
    - `node_name`: Name of the ROS 2 node (string)
    - `topic_subscriptions`: List of subscribed topics and message types (JSON/YAML)
    - `topic_publications`: List of published topics and message types (JSON/YAML)
    - `service_servers`: List of provided services (JSON/YAML)
    - `service_clients`: List of consumed services (JSON/YAML)

### Physics Engine
- **Description**: The underlying software component within Gazebo (and to some extent Unity) responsible for calculating physical behaviors such as gravity, collisions, and joint dynamics. (Often integrated within simulation environment, configuration details found there).
- **Attributes**:
    - `name`: Physics engine name (string)
    - `gravity_vector`: (x, y, z) components (tuple of floats)
    - `solver_iterations`: Number of iterations for physics solver (integer)
    - `time_step`: Simulation time step (float)
    - `collision_detection_method`: Method used for collision detection (string)

### LiDAR Sensor
- **Description**: A simulated laser-based sensor providing precise distance measurements to objects in the environment, typically represented as point clouds.
- **Attributes**:
    - `id`: Unique identifier (string)
    - `name`: Sensor name (string)
    - `type`: "LiDAR" (string)
    - `mount_point`: Relative position on robot model (x, y, z, roll, pitch, yaw)
    - `range`: (min, max) distance (tuple of floats)
    - `horizontal_fov`: Angular field of view (float)
    - `vertical_fov`: Angular field of view (float)
    - `update_rate`: Sensor data update frequency (float Hz)
    - `noise_model`: Parameters for simulated noise (JSON/YAML)
    - `ros2_topic`: ROS 2 topic for publishing data (string)

### Depth Camera Sensor
- **Description**: A simulated camera providing depth information (distance from camera to objects) for each pixel in its field of view, useful for 3D perception.
- **Attributes**:
    - `id`: Unique identifier (string)
    - `name`: Sensor name (string)
    - `type`: "DepthCamera" (string)
    - `mount_point`: Relative position on robot model (x, y, z, roll, pitch, yaw)
    - `image_resolution`: (width, height) (tuple of integers)
    - `horizontal_fov`: Angular field of view (float)
    - `update_rate`: Sensor data update frequency (float Hz)
    - `noise_model`: Parameters for simulated noise (JSON/YAML)
    - `ros2_depth_topic`: ROS 2 topic for publishing depth data (string)
    - `ros2_image_topic`: ROS 2 topic for publishing RGB image data (string, optional)

### IMU Sensor
- **Description**: A simulated Inertial Measurement Unit providing data on the robot's orientation, angular velocity, and linear acceleration.
- **Attributes**:
    - `id`: Unique identifier (string)
    - `name`: Sensor name (string)
    - `type`: "IMU" (string)
    - `mount_point`: Relative position on robot model (x, y, z, roll, pitch, yaw)
    - `update_rate`: Sensor data update frequency (float Hz)
    - `orientation_reference_frame`: (e.g., "ENU", "NED") (string)
    - `noise_model`: Parameters for simulated noise (JSON/YAML)
    - `ros2_topic`: ROS 2 topic for publishing data (string)

## Relationships

- A **Digital Twin** *has one* **Humanoid Robot Model**.
- A **Digital Twin** *can have one* **Gazebo Simulation Environment**.
- A **Digital Twin** *can have one* **Unity Simulation Environment**.
- A **Humanoid Robot Model** *can have many* **Sensors** (LiDAR, Depth Camera, IMU).
- **ROS 2** *facilitates communication between* **Simulation Environments** and **Robot Models**.