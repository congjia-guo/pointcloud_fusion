cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_fusion)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure livox_ros_driver2 message_filters pcl_conversions
  pcl_ros roscpp sensor_msgs tf
)
find_package(PCL REQUIRED COMPONENTS common filters registration)

generate_dynamic_reconfigure_options(config/Extrinsics.cfg config/Bounds.cfg)
catkin_package(CATKIN_DEPENDS
  dynamic_reconfigure livox_ros_driver2 message_filters pcl_conversions
  pcl_ros roscpp sensor_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(mid360_to_pointcloud2 src/mid360_to_pointcloud2.cpp)
add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)
add_executable(pointcloud_fusion_node src/pointcloud_fusion_node.cpp)
add_executable(multi_lidar_calibration_icp_node
  src/multi_lidar_calibration_icp.cpp src/multi_lidar_calibration_icp_node.cpp)
add_executable(multi_lidar_calibration_ndt_node
  src/multi_lidar_calibration_ndt.cpp src/multi_lidar_calibration_ndt_node.cpp)

set(nodes mid360_to_pointcloud2 pointcloud_filter_node pointcloud_fusion_node
  multi_lidar_calibration_icp_node multi_lidar_calibration_ndt_node)
foreach(node IN LISTS nodes)
  add_dependencies(${node} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${node} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endforeach()
add_dependencies(pointcloud_fusion_node ${PROJECT_NAME}_gencfg)

install(TARGETS ${nodes} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
