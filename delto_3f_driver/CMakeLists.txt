cmake_minimum_required(VERSION 3.8)
project(delto_3f_driver)

# Specify C++ Standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

# Include directories
include_directories(
  include
  include/${PROJECT_NAME}
  src
)

# Create the TCP communication library
add_library(delto_tcp SHARED
  src/delto_external_TCP.cpp
)

# Create the main driver library
add_library(${PROJECT_NAME} SHARED
  src/system_interface.cpp
  src/delto_3f_external_driver.cpp
)

# Link the TCP library to the main driver
target_link_libraries(${PROJECT_NAME}
  delto_tcp
)

# Create the executable
add_executable(delto_3f_external_driver_node 
  src/delto_3f_external_driver.cpp
)

# Link libraries to the executable
target_link_libraries(delto_3f_external_driver_node
  ${PROJECT_NAME}
  delto_tcp
)

# Add dependencies for the TCP library
ament_target_dependencies(delto_tcp
  rclcpp
)

# Add dependencies for the main driver library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  sensor_msgs
  hardware_interface
  std_msgs
  trajectory_msgs
  control_msgs
  pluginlib
)

# Add dependencies for the executable
ament_target_dependencies(delto_3f_external_driver_node
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
  pluginlib
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  delto_tcp
  delto_3f_external_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install directories
install(DIRECTORY 
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  controller 
  urdf
  cfg
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY delto_utility
  DESTINATION lib/
)

install(PROGRAMS
  ${PROJECT_NAME}/delto_3f_driver.py
  DESTINATION lib/${PROJECT_NAME}/
)

# Python package installation
ament_python_install_package(${PROJECT_NAME})

# Plugin export
pluginlib_export_plugin_description_file(hardware_interface delto_3f_driver.xml)

install(FILES delto_3f_driver.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()