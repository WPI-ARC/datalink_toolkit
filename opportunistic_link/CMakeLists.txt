cmake_minimum_required(VERSION 2.8.3)
project(opportunistic_link)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs pcl pcl_ros teleop_msgs image_transport message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  sensor_msgs
#  teleop_msgs
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opportunistic_link
  CATKIN_DEPENDS rospy std_msgs sensor_msgs pcl pcl_ros teleop_msgs image_transport message_runtime
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(opportunistic_link
#   src/${PROJECT_NAME}/opportunistic_link.cpp
# )

## Declare cpp executables

set(CMAKE_CXX_FLAGS "-g")

# Compressed image handlers
add_executable(limited_image_link_startpoint src/limited_image_link_startpoint.cpp)
add_dependencies(limited_image_link_startpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(limited_image_link_startpoint ${catkin_LIBRARIES})

add_executable(image_link_endpoint src/image_link_endpoint.cpp)
add_dependencies(image_link_endpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_link_endpoint ${catkin_LIBRARIES})

# Compressed pointcloud handlers
add_executable(limited_pointcloud_link_startpoint src/limited_pointcloud_link_startpoint.cpp)
add_dependencies(limited_pointcloud_link_startpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(limited_pointcloud_link_startpoint ${catkin_LIBRARIES} z rt)

add_executable(pointcloud_link_endpoint src/pointcloud_link_endpoint.cpp)
add_dependencies(pointcloud_link_endpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_link_endpoint ${catkin_LIBRARIES} z rt)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/opportunistic_link/hybid_link_startpoint.py
  src/opportunistic_link/limited_link_startpoint.py
  src/opportunistic_link/link_startpoint.py
  src/opportunistic_link/link_endpoint.py
  src/opportunistic_link/rate_controller.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS opportunistic_link opportunistic_link_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_opportunistic_link.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
