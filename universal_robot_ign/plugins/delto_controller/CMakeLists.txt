# ignition package
# Fortress

find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

message(STATUS "Compiling against Ignition Fortress")

find_library(HANDCONTROL_LIBRARY
  NAMES HandControl
  PATHS "."
)
# for ignition-gazebo plugin RobotiqController
add_library(DeltoController SHARED DeltoController.cpp)
add_library(HandControl SHARED HandControl.cpp)

target_link_libraries(DeltoController
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    ${HANDCONTROL_LIBRARY})




