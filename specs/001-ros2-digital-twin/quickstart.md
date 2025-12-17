# Quickstart Guide: Digital Twin Simulation Module

This quickstart guide provides a rapid overview of how to set up and interact with the digital twin simulation environment for humanoid robots, integrating Gazebo, Unity, and ROS 2. It's designed for students with basic ROS 2 knowledge to quickly get hands-on experience.

## Prerequisites

Before you begin, ensure you have the following installed and configured:

-   **ROS 2 Humble (or equivalent)**: With `colcon` build tools.
-   **Gazebo Garden (or equivalent)**: Integrated with ROS 2.
-   **Unity 2022.3 LTS (or newer)**: With the ROS-TCP-Endpoint and ROS-Unity-Integration packages.
-   **Python 3.10+**: For ROS 2 scripting and utilities.
-   **Visual Studio Code (recommended IDE)**: With relevant extensions for Python, C#, and ROS 2.

## 1. Setting Up the ROS 2 Workspace

1.  **Create a ROS 2 workspace**:
    ```bash
    mkdir -p ~/digital_twin_ws/src
    cd ~/digital_twin_ws/src
    ```
2.  **Clone the necessary ROS 2 packages**:
    ```bash
    git clone <URL_TO_YOUR_ROS2_PACKAGES_REPO>
    ```
    *(Note: This repository will contain `digital_twin_gazebo`, `digital_twin_unity`, and `humanoid_robot_description` packages.)*
3.  **Build the workspace**:
    ```bash
    cd ~/digital_twin_ws
    colcon build
    ```
4.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```
    *(Add this to your `~/.bashrc` for persistence: `echo "source ~/digital_twin_ws/install/setup.bash" >> ~/.bashrc`)*

## 2. Launching the Gazebo Simulation

The `digital_twin_gazebo` package provides launch files to start Gazebo with the humanoid robot and a basic environment.

1.  **Launch Gazebo with the humanoid robot**:
    ```bash
    ros2 launch digital_twin_gazebo humanoid_world.launch.py
    ```
    This command will open Gazebo, where you should see a humanoid robot model loaded in a simple environment. Observe its interaction with gravity and the ground plane.

2.  **Verify ROS 2 communication**:
    In a new terminal (remember to source your ROS 2 workspace), check for published topics:
    ```bash
    ros2 topic list
    ```
    You should see topics like `/joint_states`, `/tf`, and potentially sensor data topics once sensors are active.

## 3. Integrating Unity for High-Fidelity Visualization

The `digital_twin_unity` package includes a Unity project that can connect to ROS 2.

1.  **Open the Unity Project**:
    Navigate to the `digital_twin_unity/UnityProject` directory and open it with Unity Hub.
2.  **Configure ROS-TCP-Endpoint**:
    Ensure the `ROS-TCP-Endpoint` and `ROS-Unity-Integration` packages are correctly set up within Unity. Follow the in-Unity documentation for these packages.
3.  **Run the Unity Scene**:
    Open the primary scene (e.g., `Assets/Scenes/HumanoidViz.unity`) and press Play in the Unity editor.
4.  **Connect to ROS 2**:
    Unity should automatically attempt to connect to the ROS 2 core. You can verify this by checking the Unity Console for connection messages.
5.  **Observe Visualization**:
    The humanoid robot's pose and joint states from Gazebo should now be mirrored in the high-fidelity Unity environment.

## 4. Working with Simulated Sensors

The `humanoid_robot_description` package defines the robot's sensors, and `digital_twin_gazebo` can simulate them.

### LiDAR Sensor
1.  **Verify LiDAR Topic**:
    ```bash
    ros2 topic info /humanoid/laser_scan
    ```
    (Replace `/humanoid/laser_scan` with the actual LiDAR topic if different).
2.  **Visualize LiDAR Data**:
    ```bash
    rviz2
    # In rviz2, add a "LaserScan" or "PointCloud2" display and set its topic to /humanoid/laser_scan
    ```
    Move the robot in Gazebo (e.g., using `ros2 run teleop_twist_keyboard teleop_twist_keyboard`) and observe the LiDAR readings in rviz2.

### Depth Camera Sensor
1.  **Verify Depth Camera Topics**:
    ```bash
    ros2 topic list | grep depth_camera
    ```
    You should see topics like `/humanoid/depth_camera/image_raw` and `/humanoid/depth_camera/depth/image_raw`.
2.  **Visualize Depth Image**:
    ```bash
    ros2 run image_tools showimage /humanoid/depth_camera/depth/image_raw
    ```
    Observe objects at different distances in the depth image.

### IMU Sensor
1.  **Verify IMU Topic**:
    ```bash
    ros2 topic info /humanoid/imu
    ```
2.  **Monitor IMU Data**:
    ```bash
    ros2 topic echo /humanoid/imu
    ```
    Physically move or rotate the simulated robot in Gazebo (e.g., apply a force) and observe changes in acceleration and angular velocity.

## 5. Basic Human-Robot Interaction (Unity)

Within the Unity environment, you can demonstrate simple interactions.

1.  **Control Robot in Unity**:
    Implement a basic Unity script or use existing examples to send commands to the humanoid robot via ROS 2 (e.g., `/cmd_vel` topic or a custom service).
2.  **Simulate Human Presence**:
    Add a simple human avatar to your Unity scene. You can then script its movement or use a controller to demonstrate interaction scenarios, such as the robot avoiding the human.

## Next Steps

-   **Deep Dive**: Explore the code within `digital_twin_gazebo`, `digital_twin_unity`, and `humanoid_robot_description` to understand their implementation.
-   **Customization**: Modify URDF files to change robot morphology or add new sensors.
-   **Advanced Topics**: Investigate ROS 2 Navigation2 integration, AI perception, and control algorithms.