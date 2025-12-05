cmake_minimum_required(VERSION 3.8)
project(pcd_range_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters octree)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Create component library
add_library(pcd_range_publisher_component SHARED
  src/pcd_range_publisher_node.cpp
  src/pcd_mmap_reader.cpp
)

ament_target_dependencies(pcd_range_publisher_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(pcd_range_publisher_component
  ${PCL_LIBRARIES}
)

# Register the component with class_loader
rclcpp_components_register_nodes(pcd_range_publisher_component "pcd_range_publisher::PcdRangePublisherNode")

# Create standalone executable
add_executable(pcd_range_publisher_node
  src/pcd_range_publisher_node.cpp
  src/pcd_mmap_reader.cpp
)

ament_target_dependencies(pcd_range_publisher_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

target_link_libraries(pcd_range_publisher_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  pcd_range_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  pcd_range_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
