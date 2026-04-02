cmake_minimum_required(VERSION 3.14)
project(image_representation)

# To be consistent with the configuration in esvo2_core
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(dvs_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

set(ROS2_DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  builtin_interfaces
  dvs_msgs
)

# make the executable
add_executable(image_representation
    src/ImageRepresentation.cpp
    src/ImageRepresentation_node.cpp
)

target_include_directories(image_representation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(image_representation ${ROS2_DEPENDENCIES})

target_link_libraries(image_representation
   ${OpenCV_LIBRARIES}
   yaml-cpp
   glog
)

# Install executable
install(TARGETS image_representation
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
