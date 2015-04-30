cmake_minimum_required(VERSION 2.8.3)
project(opportunistic_link)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs datalink_msgs image_transport cv_bridge pointcloud_compression message_generation)
find_package(OpenCV REQUIRED)

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
catkin_package(CATKIN_DEPENDS rospy std_msgs sensor_msgs datalink_msgs image_transport cv_bridge message_runtime pointcloud_compression DEPENDS system_lib z rt OpenCV)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

## Declare a cpp library

# Image compression library
add_library(image_compression SHARED src/image_compression.cpp)
add_dependencies(image_compression ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_compression ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} z)

## Declare cpp executables

# Image resizing node
add_executable(image_resizer src/image_resizer.cpp)
add_dependencies(image_resizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_resizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Camera resizing node
add_executable(camera_resizer src/camera_resizer.cpp)
add_dependencies(camera_resizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_resizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Service-based compressed image handlers
add_executable(request_image_link_startpoint src/request_image_link_startpoint.cpp)
add_dependencies(request_image_link_startpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(request_image_link_startpoint image_compression ${catkin_LIBRARIES})

add_executable(request_image_link_endpoint src/request_image_link_endpoint.cpp)
add_dependencies(request_image_link_endpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(request_image_link_endpoint image_compression ${catkin_LIBRARIES})

# Compressed image handlers
add_executable(limited_image_link_startpoint src/limited_image_link_startpoint.cpp)
add_dependencies(limited_image_link_startpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(limited_image_link_startpoint ${catkin_LIBRARIES})

add_executable(image_link_endpoint src/image_link_endpoint.cpp)
add_dependencies(image_link_endpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_link_endpoint ${catkin_LIBRARIES})

# Service-based compressed camera handlers
add_executable(request_camera_link_startpoint src/request_camera_link_startpoint.cpp)
add_dependencies(request_camera_link_startpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(request_camera_link_startpoint image_compression ${catkin_LIBRARIES})

add_executable(request_camera_link_endpoint src/request_camera_link_endpoint.cpp)
add_dependencies(request_camera_link_endpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(request_camera_link_endpoint image_compression ${catkin_LIBRARIES})

# Compressed camera handlers
add_executable(limited_camera_link_startpoint src/limited_camera_link_startpoint.cpp)
add_dependencies(limited_camera_link_startpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(limited_camera_link_startpoint ${catkin_LIBRARIES})

add_executable(camera_link_endpoint src/camera_link_endpoint.cpp)
add_dependencies(camera_link_endpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_link_endpoint ${catkin_LIBRARIES})

# Service-based compressed pointcloud handlers
add_executable(request_pointcloud_link_startpoint src/request_pointcloud_link_startpoint.cpp)
add_dependencies(request_pointcloud_link_startpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(request_pointcloud_link_startpoint ${catkin_LIBRARIES} rt)

add_executable(request_pointcloud_link_endpoint src/request_pointcloud_link_endpoint.cpp)
add_dependencies(request_pointcloud_link_endpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(request_pointcloud_link_endpoint ${catkin_LIBRARIES} rt)

# Compressed pointcloud handlers
add_executable(limited_pointcloud_link_startpoint src/limited_pointcloud_link_startpoint.cpp)
add_dependencies(limited_pointcloud_link_startpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(limited_pointcloud_link_startpoint ${catkin_LIBRARIES} rt)

add_executable(pointcloud_link_endpoint src/pointcloud_link_endpoint.cpp)
add_dependencies(pointcloud_link_endpoint ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_link_endpoint ${catkin_LIBRARIES} rt)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  src/opportunistic_link/hybrid_link_startpoint.py
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
