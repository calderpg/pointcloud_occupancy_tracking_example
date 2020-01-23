cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_occupancy_tracking_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        sensor_msgs
                                        geometry_msgs
                                        tf2_msgs
                                        visualization_msgs
                                        message_generation
                                        common_robotics_utilities
                                        voxelized_geometry_tools)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
                  FILES
                  MultiPointCloud2.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project
catkin_package(CATKIN_DEPENDS
               roscpp
               std_msgs
               sensor_msgs
               geometry_msgs
               tf2_msgs
               visualization_msgs
               message_runtime
               common_robotics_utilities
               voxelized_geometry_tools
               DEPENDS
               Eigen3)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
                    SYSTEM ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

## Build options
add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-flto)
add_compile_options(-Werror=non-virtual-dtor)
add_compile_options(-Wold-style-cast)
add_compile_options(-march=native)

# Single-camera occupancy tracking node
add_executable(single_camera_occupancy_tracking_node
               src/single_camera_occupancy_tracking_node.cpp)
add_dependencies(single_camera_occupancy_tracking_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(single_camera_occupancy_tracking_node ${catkin_LIBRARIES})

# Multi-pointcloud aggregated occupancy tracking node
add_executable(pointcloud_occupancy_tracking_node
               src/pointcloud_occupancy_tracking_node.cpp)
add_dependencies(pointcloud_occupancy_tracking_node ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencpp)
target_link_libraries(pointcloud_occupancy_tracking_node ${catkin_LIBRARIES})

# Pointcloud aggregation node
add_executable(pointcloud_aggregator_node
               src/pointcloud_aggregator_node.cpp)
add_dependencies(pointcloud_aggregator_node ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencpp)
target_link_libraries(pointcloud_aggregator_node ${catkin_LIBRARIES})

# Install C++ nodes
install(
    TARGETS
    pointcloud_aggregator_node
    pointcloud_occupancy_tracking_node
    single_camera_occupancy_tracking_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Python node
catkin_install_python(PROGRAMS scripts/joystick_block_control.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark launch files for installation
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)

## Mark config files for installation
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    PATTERN ".svn" EXCLUDE)
