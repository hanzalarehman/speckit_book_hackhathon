cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(digital_twin_gazebo)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "srv/RobotCommand.srv"
)

ament_export_dependencies(rclpy std_msgs geometry_msgs)

ament_package()
