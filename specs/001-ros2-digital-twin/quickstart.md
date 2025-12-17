# Quickstart: Digital Twin Simulation for Humanoid Robots

This quickstart guide provides the initial steps to set up and run the digital twin simulation environment for humanoid robots using Gazebo, Unity, and ROS 2.

## Prerequisites

Before you begin, ensure you have the following installed and configured on your system:

-   **ROS 2 (Humble Hawksbill or later)**: Follow the official ROS 2 installation guide for your operating system (Ubuntu recommended).
    -   [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
-   **Gazebo (Garden or later)**: Gazebo is typically installed with ROS 2, but ensure you have the latest compatible version.
    -   [Gazebo Installation Guide](https://gazebosim.org/docs/garden/install_ubuntu)
-   **Unity Hub and Unity Editor (LTS release)**: Install Unity Hub and a Long-Term Support (LTS) version of the Unity Editor.
    -   [Unity Download Page](https://unity3d.com/get-unity/download)
-   **Unity ROS 2-Unity Bridge**: Install the ROS 2-Unity Bridge package within your Unity project.
    -   [ROS-Unity Bridge Documentation](https://github.com/Unity-Technologies/ROS-TCP-Endpoint)
-   **Python 3.x**: Ensure Python 3 is installed and configured.
-   **C++ Compiler**: A C++ compiler compatible with your ROS 2 distribution (e.g., GCC).

## Module Setup

1.  **Clone the Repository**:
    Clone the `ros2-digital-twin-module` repository to your local machine:
    ```bash
    git clone https://github.com/your-repo/ros2-digital-twin-module.git
    cd ros2-digital-twin-module
    ```

2.  **ROS 2 Workspace Setup**:
    Navigate to the ROS 2 packages directory and build the workspace:
    ```bash
    cd src/ros2_packages
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    source install/setup.bash
    ```

3.  **Unity Project Setup**:
    Open the `src/unity_project` folder in Unity Hub. Unity will import the project. Ensure the ROS 2-Unity Bridge is correctly configured within the Unity project.

4.  **Launch Gazebo Simulation**:
    To launch a basic Gazebo simulation with a humanoid robot:
    ```bash
    ros2 launch digital_twin_gazebo humanoid_robot_world.launch.py
    ```

5.  **Launch Unity Environment**:
    Open the main scene in Unity (`Assets/Scenes/MainScene.unity`) and press the Play button to start the Unity simulation. Ensure the ROS 2-Unity Bridge connection is established.

6.  **Verify ROS 2 Communication**:
    You can verify ROS 2 communication by listing active topics:
    ```bash
    ros2 topic list
    ```
    You should see topics related to robot control and sensor data (e.g., `/cmd_vel`, `/lidar_scan`, `/imu_data`).

## Next Steps

-   Explore the code examples in `src/ros2_packages` for specific functionalities like robot control and sensor data processing.
-   Refer to the detailed documentation for each chapter to deepen your understanding of physics simulation, high-fidelity environments, and sensor simulation.