# URDF for Humanoids

The Unified Robot Description Format (URDF) is an XML format used in ROS 2 to describe all elements of a robot. This includes its kinematic and dynamic properties, visual appearance, and collision geometry. While URDF can describe any robot, understanding its application to humanoid robots is crucial for developing sophisticated bipedal systems.

## Why URDF is Essential

URDF serves as a single, consistent description of your robot that can be used by various ROS 2 tools and libraries, such as:

-   **Visualization**: Tools like RViz 2 can parse a URDF file to display a 3D model of your robot.
-   **Simulation**: Simulators (though not covered in this module) can use URDF for collision detection and physics simulation.
-   **Motion Planning**: Libraries for inverse kinematics, forward kinematics, and motion planning rely on the kinematic chain defined in the URDF.
-   **Robot State Publishing**: The `robot_state_publisher` ROS 2 package reads the URDF and the current joint states to publish the full state of the robot's links and joints as a TF (Transform Frame) tree.

## Core Elements of URDF

A URDF file primarily consists of `link` and `joint` elements, connected to form a kinematic chain.

### 1. `link`: Describing a Robot's Body Parts

A `<link>` element represents a rigid body part of the robot. This could be a torso, a leg, a wheel, or even a camera housing. Each link has several optional sub-elements to define its properties:

-   **`<visual>`**: Describes the graphical properties of the link, including its geometry (box, cylinder, sphere, or mesh) and material (color, texture). This is what you see in a visualizer.
    ```xml
    <visual>
      <geometry>
        <box size="0.1 0.2 0.5" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
    ```
-   **`<collision>`**: Describes the collision properties of the link, also defining geometry. This geometry can be simpler than the visual geometry to speed up collision detection.
    ```xml
    <collision>
      <geometry>
        <box size="0.1 0.2 0.5" />
      </geometry>
    </collision>
    ```
-   **`<inertial>`**: Describes the mass, center of mass, and inertia matrix of the link. This is crucial for physics simulations and dynamic analysis.
    ```xml
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" />
    </inertial>
    ```

### 2. `joint`: Connecting Body Parts

A `<joint>` element describes the kinematic and dynamic relationship between two links. Joints define how one link (the *child* link) moves relative to another (the *parent* link).

Key attributes and sub-elements of a joint:

-   **`name`**: A unique identifier for the joint.
-   **`type`**: The type of joint (e.g., `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`).
    -   `revolute`: A rotational joint with a limited range.
    -   `continuous`: A rotational joint with an unlimited range.
    -   `prismatic`: A linear joint with a limited range.
    -   `fixed`: A joint that rigidly connects two links (no movement).
-   **`<parent link="link_name">`**: Specifies the name of the parent link.
-   **`<child link="link_name">`**: Specifies the name of the child link.
-   **`<origin xyz="x y z" rpy="roll pitch yaw">`**: Defines the transform from the parent link's origin to the child link's origin. `xyz` specifies translation, `rpy` specifies rotation in roll, pitch, and yaw.
-   **`<axis xyz="x y z">`**: For revolute and prismatic joints, defines the axis of rotation or translation.
-   **`<limit lower="val" upper="val" effort="val" velocity="val">`**: Defines the joint limits for revolute and prismatic joints.

## Minimal URDF Example: A Simple 2-Link Arm

Let's create a very simple URDF for a robot with a base and one rotating arm.

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001" />
    </inertial>
  </link>

  <!-- Joint between base and arm -->
  <joint name="base_to_arm_joint" type="revolute">
    <parent link="base_link" />
    <child link="arm_link" />
    <origin xyz="0 0 0.05" rpy="0 0 0" /> <!-- Origin at top of base -->
    <axis xyz="0 0 1" /> <!-- Rotate around Z-axis -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0" />
  </joint>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.2" />
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <material name="green">
        <color rgba="0 1 0 1" />
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.2" />
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0" />
    </collision>
    <inertial>
      <mass value="0.05" />
      <origin xyz="0 0 0.1" rpy="0 0 0" />
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001" />
    </inertial>
  </link>

  <!-- Define materials for reusability -->
  <material name="red">
    <color rgba="1 0 0 1" />
  </material>
  <material name="green">
    <color rgba="0 1 0 1" />
  </material>

</robot>
```

## Validating Your URDF

To ensure your URDF is correctly formatted and can be parsed by ROS 2 tools, you can use the `check_urdf` utility (part of the `urdfdom_py` package, often installed with ROS 2).

```bash
# First, source your ROS 2 environment
source /opt/ros/humble/setup.bash

# Then, run check_urdf on your file
check_urdf your_robot_model.urdf
```
If `check_urdf` reports no errors, your URDF's XML structure and tag usage are likely correct. For visual inspection, tools like RViz 2 are invaluable.

This minimal example provides a foundation for understanding how to build more complex URDF models for humanoid robots by combining multiple links and joints.
