cmake_minimum_required(VERSION 3.8)
project(master_the_tension_transition_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Rely on the ROS/colcon environment for runtime library discovery instead of
# embedding host-specific paths like Conda into the installed executables.
set(CMAKE_SKIP_RPATH TRUE)
set(CMAKE_SKIP_INSTALL_RPATH TRUE)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

function(workflow_demo name)
  add_executable(${name} src/${name}.cpp)
  ament_target_dependencies(
    ${name}
    geometry_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_task_constructor_core
    moveit_task_constructor_msgs
    moveit_visual_tools
    rclcpp
    sensor_msgs
    shape_msgs
    std_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
  )
  target_include_directories(${name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${name} PUBLIC c_std_99 cxx_std_17)
  install(TARGETS ${name} DESTINATION lib/${PROJECT_NAME})
endfunction()

workflow_demo(dual_mtc_routing)
workflow_demo(dual_adapt_and_test)
workflow_demo(load_scene)
workflow_demo(monitor_tracking)
workflow_demo(dlo_line_model)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/subtrajectory_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME subtrajectory_subscriber)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
