# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "custom_navigation: 1 messages, 2 services")

set(MSG_I_FLAGS "-Icustom_navigation:/home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/msg;-Isensor_msgs:/opt/ros/groovy/share/sensor_msgs/msg;-Istd_msgs:/opt/ros/groovy/share/std_msgs/msg;-Igeometry_msgs:/opt/ros/groovy/share/geometry_msgs/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

#better way to handle this?
set (ALL_GEN_OUTPUT_FILES_cpp "")

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(custom_navigation
  /home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/msg/M_ADC.msg
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_navigation
)

### Generating Services
_generate_srv_cpp(custom_navigation
  /home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/srv/S_ADC.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_navigation
)
_generate_srv_cpp(custom_navigation
  /home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/srv/S_DigitalOut.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_navigation
)

### Generating Module File
_generate_module_cpp(custom_navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(custom_navigation_gencpp ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(custom_navigation
  /home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/msg/M_ADC.msg
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_navigation
)

### Generating Services
_generate_srv_lisp(custom_navigation
  /home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/srv/S_ADC.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_navigation
)
_generate_srv_lisp(custom_navigation
  /home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/srv/S_DigitalOut.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_navigation
)

### Generating Module File
_generate_module_lisp(custom_navigation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_navigation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(custom_navigation_genlisp ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(custom_navigation
  /home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/msg/M_ADC.msg
  "${MSG_I_FLAGS}"
  "/opt/ros/groovy/share/std_msgs/msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_navigation
)

### Generating Services
_generate_srv_py(custom_navigation
  /home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/srv/S_ADC.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_navigation
)
_generate_srv_py(custom_navigation
  /home/turtlebot/Documents/rosWS/custom_ros/src/custom_navigation/srv/S_DigitalOut.srv
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_navigation
)

### Generating Module File
_generate_module_py(custom_navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(custom_navigation_genpy ALL
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)


debug_message(2 "custom_navigation: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(custom_navigation_gencpp sensor_msgs_gencpp)
add_dependencies(custom_navigation_gencpp std_msgs_gencpp)

if(genlisp_INSTALL_DIR)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_navigation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(custom_navigation_genlisp sensor_msgs_genlisp)
add_dependencies(custom_navigation_genlisp std_msgs_genlisp)

if(genpy_INSTALL_DIR)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2.7\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_navigation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(custom_navigation_genpy sensor_msgs_genpy)
add_dependencies(custom_navigation_genpy std_msgs_genpy)
