cmake_minimum_required(VERSION 3.8)
project(delto_5f_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(std_msgs REQUIRED)

add_library(delto_5f_driver SHARED
  src/system_interface.cpp
)

target_include_directories(
  delto_5f_driver
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(delto_5f_driver
  rclcpp
  hardware_interface
  pluginlib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY cfg urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS delto_5f_driver
  LIBRARY DESTINATION lib
)
pluginlib_export_plugin_description_file(hardware_interface delto_5f_driver.xml)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
