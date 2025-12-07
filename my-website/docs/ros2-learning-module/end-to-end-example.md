# Minimal End-to-End Control Example

This section provides a minimal end-to-end example demonstrating how ROS 2 nodes, topics, and services can work together to achieve a simple control task. We will create a system where a "commander" node sends a command to a "robot driver" node via a service, and the robot driver reports its status via a topic.

## Scenario: Simple Robot Movement Command

Imagine a very basic robot that can receive a command to "move forward" or "stop". We want to:
1.  Send a discrete command to the robot.
2.  Receive continuous feedback about the robot's current state (e.g., "moving" or "stopped").

## Components

We will create two main ROS 2 nodes:

1.  **`commander_node`**: This node acts as a client to send movement commands via a ROS 2 service. It will also subscribe to a status topic to receive feedback.
2.  **`robot_driver_node`**: This node provides a ROS 2 service to receive commands and publishes the robot's current status to a topic.

## 1. Define Service and Message Types

First, we need to define custom message and service types for our communication.
Create a new ROS 2 package for custom messages and services (if you don't have one already).

```bash
# Navigate to your ROS 2 workspace src directory
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake custom_msgs_srvs
cd custom_msgs_srvs
mkdir msg srv
```

### Custom Message: `RobotStatus.msg`

Create `~/ros2_ws/src/custom_msgs_srvs/msg/RobotStatus.msg`:
```
string status_message
bool is_moving
```

### Custom Service: `MovementCommand.srv`

Create `~/ros2_ws/src/custom_msgs_srvs/srv/MovementCommand.srv`:
```
string command # e.g., "move_forward", "stop"
---
bool success
string message
```

### Update `CMakeLists.txt` and `package.xml`

You need to tell ROS 2 to build these custom types.

**`~/ros2_ws/src/custom_msgs_srvs/CMakeLists.txt`**:
Add the following lines:
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/MovementCommand.srv"
)
```
Ensure `ament_cmake_ros` and `rosidl_default_generators` are found.

**`~/ros2_ws/src/custom_msgs_srvs/package.xml`**:
Add these lines within the `<build_depends>` and `<exec_depends>` sections:
```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

**Build your custom messages/services package:**
```bash
cd ~/ros2_ws
colcon build --packages-select custom_msgs_srvs
source install/setup.bash
```

## 2. `robot_driver_node` (Service Server & Publisher)

This node will provide the `MovementCommand` service and publish `RobotStatus` messages.

Create `~/ros2_ws/src/my_ros2_py_pkg/my_ros2_py_pkg/robot_driver.py`:

```python
# my_ros2_py_pkg/my_ros2_py_pkg/robot_driver.py
import rclpy
from rclpy.node import Node
from custom_msgs_srvs.msg import RobotStatus
from custom_msgs_srvs.srv import MovementCommand
import time

class RobotDriverNode(Node):
    def __init__(self):
        super().__init__('robot_driver_node')
        self.get_logger().info('RobotDriverNode has been started.')

        # Create a service server
        self.srv = self.create_service(MovementCommand, 'movement_command', self.movement_command_callback)
        self.get_logger().info('MovementCommand service is active.')

        # Create a publisher for robot status
        self.status_publisher_ = self.create_publisher(RobotStatus, 'robot_status', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.status_timer_callback)
        self.is_moving = False
        self.current_status_message = "Robot is stopped."
        self.get_logger().info('RobotStatus publisher is active.')

    def movement_command_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')
        if request.command == "move_forward":
            self.is_moving = True
            self.current_status_message = "Robot is moving forward."
            response.success = True
            response.message = "Started moving forward."
            self.get_logger().info('Robot started moving forward.')
        elif request.command == "stop":
            self.is_moving = False
            self.current_status_message = "Robot is stopped."
            response.success = True
            response.message = "Stopped robot."
            self.get_logger().info('Robot stopped.')
        else:
            response.success = False
            response.message = f"Unknown command: {request.command}"
            self.get_logger().warn(f"Unknown command received: {request.command}")
        return response

    def status_timer_callback(self):
        msg = RobotStatus()
        msg.status_message = self.current_status_message
        msg.is_moving = self.is_moving
        self.status_publisher_.publish(msg)
        self.get_logger().info(f'Publishing status: "{msg.status_message}", Moving: {msg.is_moving}')

def main(args=None):
    rclpy.init(args=args)
    robot_driver_node = RobotDriverNode()
    rclpy.spin(robot_driver_node)
    robot_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. `commander_node` (Service Client & Subscriber)

This node will send commands and listen for status updates.

Create `~/ros2_ws/src/my_ros2_py_pkg/my_ros2_py_pkg/commander.py`:

```python
# my_ros2_py_pkg/my_ros2_py_pkg/commander.py
import rclpy
from rclpy.node import Node
from custom_msgs_srvs.msg import RobotStatus
from custom_msgs_srvs.srv import MovementCommand
from rclpy.task import Future
import sys

class CommanderNode(Node):
    def __init__(self):
        super().__init__('commander_node')
        self.get_logger().info('CommanderNode has been started.')

        # Create a service client
        self.cli = self.create_client(MovementCommand, 'movement_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = MovementCommand.Request()
        self.get_logger().info('MovementCommand service client is ready.')

        # Create a subscriber for robot status
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('RobotStatus subscriber is active.')

    def status_listener_callback(self, msg):
        self.get_logger().info(f'Robot status: "{msg.status_message}", Moving: {msg.is_moving}')

    def send_command(self, command_str):
        self.request.command = command_str
        self.get_logger().info(f'Sending command: {command_str}')
        self.future = self.cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    commander_node = CommanderNode()

    # Send "move_forward" command
    response = commander_node.send_command("move_forward")
    if response.success:
        commander_node.get_logger().info(f'Command successful: {response.message}')
    else:
        commander_node.get_logger().error(f'Command failed: {response.message}')
    
    # Let it run for a bit to see status updates
    commander_node.get_logger().info('Robot is moving... (watching status for 5 seconds)')
    time.sleep(5)

    # Send "stop" command
    response = commander_node.send_command("stop")
    if response.success:
        commander_node.get_logger().info(f'Command successful: {response.message}')
    else:
        commander_node.get_logger().error(f'Command failed: {response.message}')

    commander_node.get_logger().info('Finished commands. Spinning to listen for final status.')
    # Spin indefinitely to keep receiving status messages
    try:
        rclpy.spin(commander_node)
    except KeyboardInterrupt:
        pass
    
    commander_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Update `setup.py`

Add entry points for `robot_driver` and `commander` in `~/ros2_ws/src/my_ros2_py_pkg/setup.py`:

```python
# ...
        entry_points={
            'console_scripts': [
                'simple_publisher = my_ros2_py_pkg.simple_publisher:main',
                'simple_subscriber = my_ros2_py_pkg.simple_subscriber:main',
                'robot_driver = my_ros2_py_pkg.robot_driver:main',       # ADD THIS LINE
                'commander = my_ros2_py_pkg.commander:main',             # ADD THIS LINE
            ],
        },
    )
```

## 5. Build, Source, and Run

1.  **Build Your Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_ros2_py_pkg custom_msgs_srvs
    source install/setup.bash
    ```

2.  **Run in Separate Terminals**:

    **Terminal 1 (Robot Driver)**:
    ```bash
    ros2 run my_ros2_py_pkg robot_driver
    ```

    **Terminal 2 (Commander)**:
    ```bash
    ros2 run my_ros2_py_pkg commander
    ```

    You should observe the `commander_node` sending commands, the `robot_driver_node` responding and publishing status, and the `commander_node` subscribing to and printing those status updates. This demonstrates a basic end-to-end control flow in ROS 2.
