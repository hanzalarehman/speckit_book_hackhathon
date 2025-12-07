# Python-to-ROS Control Bridges: Using `rclpy`

Python is a popular language for robotics due to its readability, extensive libraries, and rapid prototyping capabilities. ROS 2 provides `rclpy`, a Python client library that allows developers to write ROS 2 nodes and interact with the ROS 2 ecosystem seamlessly. This section will guide you through creating ROS 2 nodes in Python, focusing on publishers and subscribers to establish control bridges.

## Setting Up Your Python Environment for ROS 2

Before you begin, ensure your ROS 2 environment is sourced. You'll typically work within a ROS 2 workspace.

### Install `rclpy` (if not already installed with ROS 2)

```bash
sudo apt update
sudo apt install ros-<ros2-distro>-rclpy
# Replace <ros2-distro> with your ROS 2 distribution (e.g., humble)
```

## Creating a Simple ROS 2 Publisher Node in Python

A publisher node sends messages to a topic. Let's create a node that publishes a simple "Hello ROS 2" message.

1.  **Create a Python Package**:
    It's good practice to organize your ROS 2 Python code within a Python package inside a ROS 2 workspace.

    ```bash
    # Navigate to your ROS 2 workspace src directory
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_ros2_py_pkg --dependencies rclpy std_msgs
    ```
    This creates a new Python package named `my_ros2_py_pkg` with `rclpy` and `std_msgs` (standard message types) as dependencies.

2.  **Create the Publisher Script**:
    Inside `my_ros2_py_pkg/my_ros2_py_pkg/`, create a file named `simple_publisher.py`:

    ```python
    # my_ros2_py_pkg/my_ros2_py_pkg/simple_publisher.py
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String # Import the String message type

    class SimplePublisher(Node):
        def __init__(self):
            # Initialize the node with a name
            super().__init__('simple_publisher_node')
            
            # Create a publisher to the 'chatter' topic with String messages
            # and a queue size of 10
            self.publisher_ = self.create_publisher(String, 'chatter', 10)
            
            # Set a timer to publish messages every 0.5 seconds
            timer_period = 0.5  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            
            self.i = 0 # Counter for messages
            self.get_logger().info('SimplePublisher node has been started.')

        def timer_callback(self):
            # Create a String message
            msg = String()
            msg.data = f'Hello ROS 2! Count: {self.i}'
            
            # Publish the message
            self.publisher_.publish(msg)
            
            # Log the published message
            self.get_logger().info(f'Publishing: "{msg.data}"')
            self.i += 1

    def main(args=None):
        # Initialize rclpy library
        rclpy.init(args=args)
        
        # Create the publisher node
        simple_publisher = SimplePublisher()
        
        # Spin the node to process callbacks (e.g., timer callback)
        rclpy.spin(simple_publisher)
        
        # Destroy the node once rclpy.spin() is stopped
        simple_publisher.destroy_node()
        
        # Shutdown rclpy library
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

3.  **Update `setup.py`**:
    To make your Python script runnable as a ROS 2 executable, you need to add an entry point in your `setup.py` file. Open `~/ros2_ws/src/my_ros2_py_pkg/setup.py` and add the following inside the `entry_points` dictionary:

    ```python
    # ... other imports
    import os
    from glob import glob
    from setuptools import find_packages, setup

    package_name = 'my_ros2_py_pkg'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yem]'))),
            (os.path.join('share', package_name), glob('launch/*.py')) # Add launch files
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='your_name', # Replace with your name
        maintainer_email='your_email@example.com', # Replace with your email
        description='A simple ROS 2 Python package',
        license='Apache-2.0', # Or appropriate license
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'simple_publisher = my_ros2_py_pkg.simple_publisher:main', # ADD THIS LINE
            ],
        },
    )
    ```

4.  **Build Your Workspace**:
    ```bash
    # Navigate to your ROS 2 workspace root
    cd ~/ros2_ws
    colcon build --packages-select my_ros2_py_pkg
    ```

5.  **Source Your Workspace**:
    ```bash
    # After building, source the setup files to make your new package available
    source install/setup.bash
    ```

6.  **Run the Publisher Node**:
    ```bash
    ros2 run my_ros2_py_pkg simple_publisher
    ```
    You should see messages being published to the console.

## Creating a Simple ROS 2 Subscriber Node in Python

A subscriber node receives messages from a topic. Let's create a node that listens to the `chatter` topic from our publisher.

1.  **Create the Subscriber Script**:
    Inside `~/ros2_ws/src/my_ros2_py_pkg/my_ros2_py_pkg/`, create a file named `simple_subscriber.py`:

    ```python
    # my_ros2_py_pkg/my_ros2_py_pkg/simple_subscriber.py
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimpleSubscriber(Node):
        def __init__(self):
            super().__init__('simple_subscriber_node')
            
            # Create a subscriber to the 'chatter' topic with String messages
            self.subscription = self.create_subscription(
                String,
                'chatter',
                self.listener_callback,
                10)
            self.subscription  # prevent unused variable warning
            self.get_logger().info('SimpleSubscriber node has been started.')

        def listener_callback(self):
            # Log the received message
            self.get_logger().info(f'I heard: "{msg.data}"')

    def main(args=None):
        rclpy.init(args=args)
        
        simple_subscriber = SimpleSubscriber()
        
        # Spin the node to process callbacks (e.g., subscription callback)
        rclpy.spin(simple_subscriber)
        
        simple_subscriber.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

2.  **Update `setup.py`**:
    Add another entry point for the subscriber in `~/ros2_ws/src/my_ros2_py_pkg/setup.py`:

    ```python
    # ...
            'console_scripts': [
                'simple_publisher = my_ros2_py_pkg.simple_publisher:main',
                'simple_subscriber = my_ros2_py_pkg.simple_subscriber:main', # ADD THIS LINE
            ],
    # ...
    ```

3.  **Re-build and Source Your Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select my_ros2_py_pkg
    source install/setup.bash
    ```

4.  **Run the Subscriber Node**:
    Open a **new terminal**, source your workspace, and run:
    ```bash
    ros2 run my_ros2_py_pkg simple_subscriber
    ```
    In a **separate terminal**, run the publisher:
    ```bash
    ros2 run my_ros2_py_pkg simple_publisher
    ```
    You should see the subscriber terminal printing the messages published by the publisher.

This concludes the basic setup for Python-to-ROS control bridges using `rclpy` for publisher and subscriber nodes.
