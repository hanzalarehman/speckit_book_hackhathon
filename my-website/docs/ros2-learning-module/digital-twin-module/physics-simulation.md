# Physics Simulation in Gazebo

This chapter delves into the fundamental aspects of physics simulation within Gazebo, focusing on how to achieve realistic behavior for humanoid robots. We will cover gravity, collisions, and the dynamics of robot joints and linkages. Understanding these concepts is crucial for developing robust and reliable robotic systems in a simulated environment.

## Gravity Simulation

Gravity is a fundamental force that dictates how objects behave in a physical world. In Gazebo, gravity is simulated to mimic real-world conditions, causing objects to fall, rest on surfaces, and interact realistically.

To observe gravity in action:
1.  Launch a Gazebo world with a humanoid robot model (as defined in `humanoid_world.launch.py`).
2.  The robot, if not properly supported, will fall to the ground plane due to simulated gravity.
3.  Experiment with placing the robot on an elevated surface. When released, it should fall.

## Realistic Collisions

Collisions are crucial for accurate robot simulation, preventing objects from passing through each other (interpenetration). Gazebo uses a physics engine to detect and resolve collisions, applying appropriate forces to simulate contact.

**Key Concepts**:
-   **Collision Meshes**: These are simplified geometric representations of a robot's links and environmental objects, used by the physics engine for collision detection. They are typically less detailed than visual meshes to reduce computational overhead.
-   **Friction**: Properties like friction coefficients (`mu`, `mu2`) defined in the SDF/URDF determine how surfaces slide or grip against each other.
-   **Restitution**: This property dictates how "bouncy" a collision is.

To demonstrate collisions:
1.  Launch `humanoid_world.launch.py`, which includes a `collision_box`.
2.  Use a ROS 2 teleoperation tool (e.g., `teleop_twist_keyboard`) to move your humanoid robot towards the `collision_box`.
3.  Observe how the robot interacts with the box: it should push against it, or if it has sufficient mass and velocity, it might move the box. The robot should not pass through the box.

## Robot Dynamics: Joints and Actuation

Robot dynamics concern how forces and torques cause motion. In Gazebo, the kinematics (positions and orientations) and dynamics (mass, inertia, forces) of each link and joint are modeled.

**Key Concepts**:
-   **Joints**: Connect links and define their relative motion (e.g., revolute, prismatic).
-   **Actuators**: The components that generate force or torque to move the joints (e.g., motors). In simulation, these are often controlled through ROS 2 interfaces.
-   **Inertia**: A measure of an object's resistance to changes in its state of motion. Defined by mass and inertia tensor in URDF.

**Example: Joint Control via ROS 2**

You can send commands to the robot's joints via ROS 2 topics. For example, to control a joint, you would typically publish to a `/joint_group_controller/commands` topic or similar, depending on the specific `ros2_control` setup.

Here's a conceptual Python example for sending a command to a joint (assuming a `JointState` message type and a publisher is set up in `joint_controller.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander')
        self.publisher_ = self.create_publisher(Float64, '/simple_humanoid/joint_name/command', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):
        msg = Float64()
        msg.data = 0.5 * (1 + (self.i % 2)) # Toggle between 0.5 and 1.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_commander = JointCommander()
    rclpy.spin(joint_commander)
    joint_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

This example demonstrates how a simple ROS 2 node can be used to send commands to a specific joint. You would map this to your robot's actual joint names and controller topics.

## Troubleshooting Physics Issues

-   **Robot Instability**: Often caused by poorly defined joint limits, incorrect inertia properties in the URDF, or high friction values.
-   **Interpenetration**: Usually indicates a problem with collision mesh definitions or physics engine settings. Ensure collision meshes are correctly defined and distinct from visual meshes.
-   **Unrealistic Movement**: Check joint motor parameters, PID gains (if using position/velocity controllers), and ensure the physics update rate is appropriate.

By understanding and carefully configuring these physics parameters, you can achieve highly realistic and valuable simulations in Gazebo for your humanoid robot.