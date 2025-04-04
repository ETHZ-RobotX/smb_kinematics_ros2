cmake_minimum_required(VERSION 3.8)
project(smb_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add executable and specify sources
add_executable(smb_kinematics_node
  # src/smb_kinematics_node.cpp
  src/smb_differential_drive_controller.cpp
)

# Specify include directories
target_include_directories(smb_kinematics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries
ament_target_dependencies(smb_kinematics_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
)

# Install targets
install(TARGETS smb_kinematics_node
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
