cmake_minimum_required(VERSION 3.8)
project(flame_ros)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(flame REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  image_transport
  tf2_ros
  sensor_msgs
  geometry_msgs
  cv_bridge
  mrs_lib
  nav_msgs
  pcl_msgs
  pcl_conversions
  flame_ros_msgs
  Eigen3
  )

set(LIBRARIES
  FlameRos_Flame
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(Sophus_INCLUDE_DIRS /opt/ros/jazzy/include)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${flame_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

## | ----------------------- Flame ROS ---------------------- |

add_library(FlameRos_Flame SHARED
  src/flame_ros.cpp
  src/tracked_image_stream.cpp
  src/utils.cpp
)

rclcpp_components_register_node(
    FlameRos_Flame
    PLUGIN "flame_ros::FlameRos"
    EXECUTABLE ${PROJECT_NAME}
)

# Apply dependencies to the target
ament_target_dependencies(FlameRos_Flame
  ${DEPENDENCIES}
)

# Link OpenCV libraries
target_link_libraries(FlameRos_Flame
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${flame_LIBRARIES}
  ${Sophus_LIBRARIES}
)

## --------------------------------------------------------------
## |                       install                              |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
  )

install(TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY tmux
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
  )

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
