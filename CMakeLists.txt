cmake_minimum_required(VERSION 3.4)
project(fluxvla_franka_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
find_package(ruckig REQUIRED)

add_message_files(FILES ZeroJacobian.msg)
generate_messages()

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fluxvla_franka_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

add_library(fluxvla_franka_controllers
  src/joint_position_controller.cpp
  src/cartesian_impedance_controller.cpp
  src/ruckig_joint_position_controller.cpp
  src/ruckig_joint_impedance_controller.cpp
  )

add_dependencies(fluxvla_franka_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(fluxvla_franka_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  ruckig::ruckig
)

target_include_directories(fluxvla_franka_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(fluxvla_franka_controllers PUBLIC
  include
)

## Installation
install(TARGETS fluxvla_franka_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/replay_joint_impedance_csv.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES fluxvla_franka_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES LICENSE NOTICE README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
  RESULT_VARIABLE CLANG_TOOLS
)
if(CLANG_TOOLS)
  file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  file(GLOB_RECURSE HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
  )
  add_format_target(fluxvla_franka_controllers FILES ${SOURCES} ${HEADERS})
  add_tidy_target(fluxvla_franka_controllers
    FILES ${SOURCES}
    DEPENDS fluxvla_franka_controllers
  )
endif()
