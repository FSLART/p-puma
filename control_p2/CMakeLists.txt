cmake_minimum_required(VERSION 3.8)
project(control_p2)

set(library_name control_p2)
set(lib_srcs
  src/control_node.cpp
  src/control_manager.cpp
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lart_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../lart_common
)

add_library(${library_name} ${lib_srcs})
ament_target_dependencies(${library_name} rclcpp lart_msgs tf2 tf2_geometry_msgs)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp lart_msgs tf2 tf2_geometry_msgs)

target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_SOURCE_DIR}/../../lart_common
)

target_link_libraries(control_node ${library_name})

install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
