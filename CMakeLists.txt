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
  Flame
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

add_executable(Flame
  src/flame_ros.cpp
  src/flame_node.cpp
)

ament_target_dependencies(Flame
  ${DEPENDENCIES}
)

## --------------------------------------------------------------
## |                       install                              |
## --------------------------------------------------------------

install(TARGETS ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include/${PROJECT_NAME}
)

# install(
#   DIRECTORY launch config
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_export_include_directories(
#   include
#   )

# ament_export_targets(
#   export_${PROJECT_NAME} HAS_LIBRARY_TARGET
#   )

# ament_export_dependencies(
#   ${DEPENDENCIES}
#   )

ament_package()
