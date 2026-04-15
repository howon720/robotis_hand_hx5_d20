cmake_minimum_required(VERSION 3.8)
project(robotis_hand_tactile)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robotis_interfaces REQUIRED)
set(ROS2_PACKAGES
    rclcpp
    std_msgs
    geometry_msgs
    visualization_msgs
    control_msgs
    sensor_msgs
    trajectory_msgs
    robotis_interfaces)

add_executable(tactile_force_rviz src/tactile_force_rviz.cpp)
target_include_directories(tactile_force_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tactile_force_rviz ${ROS2_PACKAGES})
install(TARGETS tactile_force_rviz
        DESTINATION lib/${PROJECT_NAME})

add_executable(reactive_force src/reactive_force.cpp)
target_include_directories(reactive_force PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(reactive_force ${ROS2_PACKAGES})
install(TARGETS reactive_force
        DESTINATION lib/${PROJECT_NAME})

add_executable(reactive_force_region src/reactive_force_region.cpp)
target_include_directories(reactive_force_region PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(reactive_force_region ${ROS2_PACKAGES})
install(TARGETS reactive_force_region
        DESTINATION lib/${PROJECT_NAME})

add_executable(tactile_grasp_controller_node 
              src/tactile_grasp_controller_node.cpp
              src/tactile_grasp_controller.cpp
              src/tactile_sensor_processor.cpp
              src/tactile_correction_planner.cpp
              src/tactile_grasp_controller_init.cpp
              src/finger_ik_solver.cpp)
target_include_directories(tactile_grasp_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tactile_grasp_controller_node ${ROS2_PACKAGES})
install(TARGETS tactile_grasp_controller_node
        DESTINATION lib/${PROJECT_NAME})

add_executable(tactile_grasp_controller_hold 
              src/tactile_grasp_controller_hold.cpp
              src/tactile_sensor_processor.cpp)
target_include_directories(tactile_grasp_controller_hold PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(tactile_grasp_controller_hold ${ROS2_PACKAGES})
install(TARGETS tactile_grasp_controller_hold
        DESTINATION lib/${PROJECT_NAME})

add_executable(finger_ik_testbench
              src/finger_ik_testbench.cpp
              src/finger_ik_solver.cpp)
target_include_directories(finger_ik_testbench PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(finger_ik_testbench ${ROS2_PACKAGES})
install(TARGETS finger_ik_testbench
        DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/finger_pressure.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()