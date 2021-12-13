cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  color_detector_msgs
  color_detector_srvs
  color_detector_params
  dynamixel_angle_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  Target.msg
  TargetArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(ellipse_visualizer src/ellipse_visualizer.cpp)

add_dependencies(ellipse_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ellipse_visualizer
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(target_tracker src/target_tracker_node.cpp src/target_tracker.cpp)

add_dependencies(target_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(target_tracker
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)
