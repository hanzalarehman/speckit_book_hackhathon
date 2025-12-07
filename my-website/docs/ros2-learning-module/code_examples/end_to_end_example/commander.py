import rclpy
from rclpy.node import Node
from custom_msgs_srvs.msg import RobotStatus
from custom_msgs_srvs.srv import MovementCommand
from rclpy.task import Future
import sys
import time

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
