cmake_minimum_required(VERSION 3.14)
project(esvo2_core)

# explicitly set std=c++14 to remove errors from pcl library
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(events_repacking_tool REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(minkindr REQUIRED)
find_package(minkindr_conversions REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(Ceres REQUIRED)

set(ROS2_DEPENDENCIES
  rclcpp
  dvs_msgs
  events_repacking_tool
  cv_bridge
  image_geometry
  image_transport
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  builtin_interfaces
  message_filters
  minkindr
  minkindr_conversions
)

###########
## Build ##
###########

set(HEADERS
  include/esvo2_core/core/DepthFusion.h
  include/esvo2_core/core/DepthRegularization.h
  include/esvo2_core/core/DepthProblem.h
  include/esvo2_core/core/DepthProblemSolver.h
  include/esvo2_core/core/EventBM.h
  include/esvo2_core/core/RegProblemLM.h
  include/esvo2_core/core/RegProblemSolverLM.h
  include/esvo2_core/core/BackendOptimization.h
  include/esvo2_core/factor/OptimizationFunctor.h
  include/esvo2_core/container/CameraSystem.h
  include/esvo2_core/container/DepthPoint.h
  include/esvo2_core/container/EventMatchPair.h
  include/esvo2_core/container/SmartGrid.h
  include/esvo2_core/container/DepthMap.h
  include/esvo2_core/container/ResidualItem.h
  include/esvo2_core/container/TimeSurfaceObservation.h
  include/esvo2_core/tools/Visualization.h
  include/esvo2_core/tools/utils.h
  include/esvo2_core/tools/TicToc.h
  include/esvo2_core/tools/sobel.h
  include/esvo2_core/tools/cayley.h
  include/esvo2_core/tools/params_helper.h
  include/esvo2_core/factor/utility.h
  include/esvo2_core/factor/pose_local_parameterization.h
)

set(SOURCES
  src/core/DepthFusion.cpp
  src/core/DepthRegularization.cpp
  src/core/DepthProblem.cpp
  src/core/DepthProblemSolver.cpp
  src/core/EventBM.cpp
  src/core/RegProblemLM.cpp
  src/core/RegProblemSolverLM.cpp
  src/core/BackendOptimization.cpp
  src/container/CameraSystem.cpp
  src/container/DepthPoint.cpp
  src/container/ResidualItem.cpp
  src/tools/Visualization.cpp
  src/tools/sobel.cpp
  src/tools/cayley.cpp
)

# Library
add_library(${PROJECT_NAME}_LIB ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME}_LIB PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_LIB ${ROS2_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_LIB
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  yaml-cpp
  glog
)

# Node esvo2_Mapping
add_executable(esvo2_Mapping
  src/esvo2_MappingNode.cpp
  src/esvo2_Mapping.cpp
  include/esvo2_core/esvo2_Mapping.h)
target_include_directories(esvo2_Mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)
ament_target_dependencies(esvo2_Mapping ${ROS2_DEPENDENCIES})
target_link_libraries(esvo2_Mapping
  ${PROJECT_NAME}_LIB
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  yaml-cpp
  glog
)

# Node esvo2_Tracking
add_executable(esvo2_Tracking
  src/esvo2_TrackingNode.cpp
  src/esvo2_Tracking.cpp
  include/esvo2_core/esvo2_Tracking.h)
target_include_directories(esvo2_Tracking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(esvo2_Tracking ${ROS2_DEPENDENCIES})
target_link_libraries(esvo2_Tracking
  ${PROJECT_NAME}_LIB
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
  glog
)

# Install
install(TARGETS
  ${PROJECT_NAME}_LIB
  esvo2_Mapping
  esvo2_Tracking
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

install(DIRECTORY calib/
  DESTINATION share/${PROJECT_NAME}/calib
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install rviz and perspective configs
install(FILES
  esvo2_system.rviz
  esvo2_system_DSEC.rviz
  esvo2_mvstereo.rviz
  esvo2_system.perspective
  esvo2_system_DSEC.perspective
  esvo2_mvstereo.perspective
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
