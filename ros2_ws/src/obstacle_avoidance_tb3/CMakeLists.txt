cmake_minimum_required(VERSION 3.5)
project(obstacle_avoidance_tb3)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(
    DIRECTORY scripts/
    DESTINATION lib/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
)

# This must be the last statement in the file
ament_package()

