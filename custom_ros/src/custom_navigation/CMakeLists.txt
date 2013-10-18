cmake_minimum_required(VERSION 2.8.3)
project(custom_navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp sensor_msgs std_msgs nav_msgs message_filters tf pcl pcl_ros)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  M_ADC.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  S_ADC.srv
  S_DigitalOut.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES custom_navigation
   CATKIN_DEPENDS message_generation roscpp sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(adcClient src/adcClient.cpp)
target_link_libraries(adcClient ${catkin_LIBRARIES})
add_dependencies(adcClient custom_navigation_gencpp)

add_executable(adcDriver src/adcDriver.cpp)
target_link_libraries(adcDriver ${catkin_LIBRARIES})
add_dependencies(adcDriver custom_navigation_gencpp)

add_executable(adcParams src/adcParams.cpp)
target_link_libraries(adcParams ${catkin_LIBRARIES})
add_dependencies(adcParams custom_navigation_gencpp)

add_executable(odom_gyro src/odom_gyro.cpp)
target_link_libraries(odom_gyro ${catkin_LIBRARIES})
add_dependencies(odom_gyro custom_navigation_gencpp)

add_executable(gyro_calibration src/gyro_calibration.cpp)
target_link_libraries(gyro_calibration ${catkin_LIBRARIES})
add_dependencies(gyro_calibration custom_navigation_gencpp)

add_executable(robot_mover src/robot_mover.cpp)
target_link_libraries(robot_mover ${catkin_LIBRARIES})
add_dependencies(robot_mover custom_navigation_gencpp)

add_executable(robot_obstacle src/robot_obstacle.cpp)
target_link_libraries(robot_obstacle ${catkin_LIBRARIES})
add_dependencies(robot_obstacle custom_navigation_gencpp)

add_executable(robot_position src/robot_position.cpp)
target_link_libraries(robot_position ${catkin_LIBRARIES})
add_dependencies(robot_position custom_navigation_gencpp)

add_executable(random_position src/random_position.cpp)
target_link_libraries(random_position ${catkin_LIBRARIES})
add_dependencies(random_position custom_navigation_gencpp)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker custom_navigation_gencpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener custom_navigation_gencpp)
