cmake_minimum_required(VERSION 3.14)
project(dv_ros_msgs VERSION 1.2.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Event.msg
  msg/EventArray.msg
  msg/Trigger.msg
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
