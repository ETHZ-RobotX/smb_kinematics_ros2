cmake_minimum_required(VERSION 3.8)
project(smb_kinematics_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable and specify sources
add_executable(smb_kinematics_ros2_node
  # src/smb_kinematics_ros2_node.cpp
  src/smb_differential_drive_controller.cpp
)

add_executable(smb_cmd_vel
  src/joy_to_cmd_vel.cpp
)

# Specify include directories
target_include_directories(smb_kinematics_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(smb_cmd_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
ament_target_dependencies(smb_kinematics_ros2_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
)

ament_target_dependencies(smb_cmd_vel
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)

# Install targets
install(TARGETS smb_kinematics_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS smb_cmd_vel
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# # Install header files
# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
