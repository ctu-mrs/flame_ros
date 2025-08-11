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
find_package(rviz_common REQUIRED)

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
  pluginlib
  )

set(EXECUTABLES
  flame_ros
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# Handle Eigen3 and OpenCV separately (like in the working example)
#find_package(Eigen3 REQUIRED)
# pkg_check_modules(EIGEN3 REQUIRED eigen3>=3.2)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OGRE REQUIRED)

set(Sophus_INCLUDE_DIRS /opt/ros/jazzy/include)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
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

add_library(${PROJECT_NAME}_component SHARED
  src/flame_ros.cpp
  src/tracked_image_stream.cpp
  src/utils.cpp
)

rclcpp_components_register_node(
    ${PROJECT_NAME}_component
    PLUGIN "flame_ros::FlameRos"
    EXECUTABLE ${PROJECT_NAME}
)

# Apply dependencies to the target
ament_target_dependencies(${PROJECT_NAME}_component
  ${DEPENDENCIES}
  Eigen3
)

# Link OpenCV libraries
target_link_libraries(${PROJECT_NAME}_component
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${flame_LIBRARIES}  # ← Use variables instead
  ${Sophus_LIBRARIES}
)

## | ----------------- RViz plugins ----------------- |

# pkg_check_modules(OGRE OGRE)

# ## This plugin includes Qt widgets, so we must include Qt like so:
# ## We'll use the version that rviz used so they are compatible.
# set(CMAKE_AUTOMOC ON) # This setting causes Qt's "MOC" generation to happen automatically.
# if(rviz_QT_VERSION VERSION_LESS "5")
#   message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
#   ## pull in all required include dirs, define QT_LIBRARIES, etc.
#   include(${QT_USE_FILE})
# else()
#   message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
#   find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
#   ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
#   set(QT_LIBRARIES Qt5::Widgets)
# endif()

# ## Prefer the Qt signals and slots to avoid defining "emit", "slots",
# ## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
# add_definitions(-DQT_NO_KEYWORDS)

# include_directories(${OGRE_INCLUDE_DIRS})
# link_directories(${OGRE_LIBRARY_DIRS})

add_library(textured_mesh_display SHARED
  src/flame_rviz_plugins/textured_mesh_display.cpp
  src/flame_rviz_plugins/textured_mesh_visual.cpp
  src/flame_rviz_plugins/surface_normals_visual.cpp
)

ament_target_dependencies(textured_mesh_display
  ${DEPENDENCIES}
  Eigen3
  pluginlib
  rviz_common
)

target_link_libraries(textured_mesh_display
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${flame_LIBRARIES}  # ← Use variables instead
  ${Sophus_LIBRARIES}
)

## --------------------------------------------------------------
## |                       install                              |
## --------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}_component
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

install(FILES rviz_common_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  ${DEPENDENCIES}
  Eigen3
)

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()