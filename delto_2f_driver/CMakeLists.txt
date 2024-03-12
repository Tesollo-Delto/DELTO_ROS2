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


# Include C++ directories
include_directories(include)
include_directories(include/delto_3f_driver)
include_directories(src)
# Declare a C++ library
add_library(delto_3f_driver src/delto_3f_external_driver.cpp)

# Declare a C++ executable
add_executable(delto_3f_external_driver_node src/delto_3f_external_driver.cpp)

# Specify libraries to link a library or executable target against


target_include_directories(delto_3f_external_driver_node 
  PUBLIC
  /include
  /include/delto_3f_driver
)
target_link_libraries(delto_3f_external_driver_node ${PROJECT_NAME})

ament_target_dependencies(delto_3f_external_driver_node
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
)
ament_target_dependencies(delto_3f_driver
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  trajectory_msgs
  control_msgs
)

# Install C++ executables
install(TARGETS
  delto_3f_external_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY include
  DESTINATION include/
)


install(DIRECTORY delto_utility
  DESTINATION lib/
)


install(DIRECTORY
  launch controller
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  ${PROJECT_NAME}/delto_3f_driver.py
  DESTINATION lib/${PROJECT_NAME}/
)


target_include_directories(delto_3f_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set (SOURCES src/delto_3f_external_driver.cpp)
set (HEADERS include/delto_3f_driver/delto_3f_external_driver.hpp )
ament_python_install_package(${PROJECT_NAME}) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# Export dependencies
ament_package()