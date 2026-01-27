cmake_minimum_required(VERSION 3.5)
project(robot_mover)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define the custom interfaces
set(rosidl_generate_interfaces_DEPENDENCIES
  std_msgs
  geometry_msgs
)

# Generate the interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Rotate.srv"
  "srv/PauseRotation.srv"
  DEPENDENCIES ${rosidl_generate_interfaces_DEPENDENCIES}
)

# Note: Python modules are handled by setup.py in ament_python fashion
# We don't use ament_python_install_package here to avoid conflicts

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()