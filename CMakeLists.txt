cmake_minimum_required(VERSION 3.8)
project(flame_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  ament_cmake
  )

set(EXECUTABLES
  flame_ros
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

## | ----------------------- Flame ROS ---------------------- |

add_library(${PROJECT_NAME}_component SHARED
  src/flame_ros.cpp
)

rclcpp_components_register_node(
    ${PROJECT_NAME}_component
    PLUGIN "flame_ros::FlameRos"
    EXECUTABLE ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_component
  ${DEPENDENCIES}
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
  DIRECTORY launch
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
)

ament_package()
