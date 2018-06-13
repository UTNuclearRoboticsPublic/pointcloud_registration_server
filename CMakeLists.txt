cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_registration_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions 
  pcl_ros 
  roscpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs 
  message_generation
  pointcloud_processing_server
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  registration_service.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs pointcloud_processing_server
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reg_creation pointcloud_registration
  CATKIN_DEPENDS pointcloud_processing_server
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(registration_example_client src/registration_example_client.cpp)
target_link_libraries( 
  registration_example_client reg_creation
  ${catkin_LIBRARIES}
)
add_dependencies(
  registration_example_client
  ${pointcloud_registration_server_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

add_executable(registration_server src/registration_server.cpp)
target_link_libraries( 
  registration_server pointcloud_registration
  ${catkin_LIBRARIES}
)
add_dependencies(
  registration_server
  ${pointcloud_registration_server_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

add_library(pointcloud_registration src/pointcloud_registration.cpp )
target_link_libraries( 
  pointcloud_registration
  ${catkin_LIBRARIES}
)
add_dependencies(pointcloud_registration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(reg_creation src/reg_creation.cpp )
target_link_libraries( 
  reg_creation
  ${catkin_LIBRARIES}
)
add_dependencies(reg_creation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)