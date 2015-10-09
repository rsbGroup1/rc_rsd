# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rc_grasp: 0 messages, 9 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rc_grasp_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Move.srv" NAME_WE)
add_custom_target(_rc_grasp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc_grasp" "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Move.srv" ""
)

get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/setConfiguration.srv" NAME_WE)
add_custom_target(_rc_grasp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc_grasp" "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/setConfiguration.srv" ""
)

get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Open.srv" NAME_WE)
add_custom_target(_rc_grasp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc_grasp" "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Open.srv" ""
)

get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getConfiguration.srv" NAME_WE)
add_custom_target(_rc_grasp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc_grasp" "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getConfiguration.srv" ""
)

get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/grabBrick.srv" NAME_WE)
add_custom_target(_rc_grasp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc_grasp" "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/grabBrick.srv" ""
)

get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Stop.srv" NAME_WE)
add_custom_target(_rc_grasp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc_grasp" "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Stop.srv" ""
)

get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/stopRobot.srv" NAME_WE)
add_custom_target(_rc_grasp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc_grasp" "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/stopRobot.srv" ""
)

get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getIsMoving.srv" NAME_WE)
add_custom_target(_rc_grasp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc_grasp" "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getIsMoving.srv" ""
)

get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getQueueSize.srv" NAME_WE)
add_custom_target(_rc_grasp_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rc_grasp" "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getQueueSize.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
)
_generate_srv_cpp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/setConfiguration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
)
_generate_srv_cpp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Open.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
)
_generate_srv_cpp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getConfiguration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
)
_generate_srv_cpp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/grabBrick.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
)
_generate_srv_cpp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Stop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
)
_generate_srv_cpp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getIsMoving.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
)
_generate_srv_cpp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/stopRobot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
)
_generate_srv_cpp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getQueueSize.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
)

### Generating Module File
_generate_module_cpp(rc_grasp
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rc_grasp_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rc_grasp_generate_messages rc_grasp_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Move.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_cpp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/setConfiguration.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_cpp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Open.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_cpp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getConfiguration.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_cpp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/grabBrick.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_cpp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Stop.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_cpp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/stopRobot.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_cpp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getIsMoving.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_cpp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getQueueSize.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_cpp _rc_grasp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rc_grasp_gencpp)
add_dependencies(rc_grasp_gencpp rc_grasp_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rc_grasp_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
)
_generate_srv_lisp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/setConfiguration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
)
_generate_srv_lisp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Open.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
)
_generate_srv_lisp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getConfiguration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
)
_generate_srv_lisp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/grabBrick.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
)
_generate_srv_lisp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Stop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
)
_generate_srv_lisp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getIsMoving.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
)
_generate_srv_lisp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/stopRobot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
)
_generate_srv_lisp(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getQueueSize.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
)

### Generating Module File
_generate_module_lisp(rc_grasp
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rc_grasp_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rc_grasp_generate_messages rc_grasp_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Move.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_lisp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/setConfiguration.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_lisp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Open.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_lisp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getConfiguration.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_lisp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/grabBrick.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_lisp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Stop.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_lisp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/stopRobot.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_lisp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getIsMoving.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_lisp _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getQueueSize.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_lisp _rc_grasp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rc_grasp_genlisp)
add_dependencies(rc_grasp_genlisp rc_grasp_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rc_grasp_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
)
_generate_srv_py(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/setConfiguration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
)
_generate_srv_py(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Open.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
)
_generate_srv_py(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getConfiguration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
)
_generate_srv_py(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/grabBrick.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
)
_generate_srv_py(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Stop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
)
_generate_srv_py(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getIsMoving.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
)
_generate_srv_py(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/stopRobot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
)
_generate_srv_py(rc_grasp
  "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getQueueSize.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
)

### Generating Module File
_generate_module_py(rc_grasp
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rc_grasp_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rc_grasp_generate_messages rc_grasp_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Move.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_py _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/setConfiguration.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_py _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Open.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_py _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getConfiguration.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_py _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/grabBrick.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_py _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/Stop.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_py _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/stopRobot.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_py _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getIsMoving.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_py _rc_grasp_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/student/catkin_ws/src/rc_rsd/RC_Grasp/srv/getQueueSize.srv" NAME_WE)
add_dependencies(rc_grasp_generate_messages_py _rc_grasp_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rc_grasp_genpy)
add_dependencies(rc_grasp_genpy rc_grasp_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rc_grasp_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rc_grasp
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(rc_grasp_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rc_grasp
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(rc_grasp_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rc_grasp
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(rc_grasp_generate_messages_py std_msgs_generate_messages_py)
