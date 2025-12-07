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
