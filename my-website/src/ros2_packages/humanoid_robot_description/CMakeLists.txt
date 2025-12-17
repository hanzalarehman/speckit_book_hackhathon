cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(humanoid_robot_description)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install files for the description
install(
  DIRECTORY launch urdf rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(urdf xacro)

ament_package()
