# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "apriltag_finder: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(apriltag_finder_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv" NAME_WE)
add_custom_target(_apriltag_finder_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "apriltag_finder" "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(apriltag_finder
  "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltag_finder
)

### Generating Module File
_generate_module_cpp(apriltag_finder
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltag_finder
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(apriltag_finder_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(apriltag_finder_generate_messages apriltag_finder_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv" NAME_WE)
add_dependencies(apriltag_finder_generate_messages_cpp _apriltag_finder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_finder_gencpp)
add_dependencies(apriltag_finder_gencpp apriltag_finder_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_finder_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(apriltag_finder
  "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltag_finder
)

### Generating Module File
_generate_module_eus(apriltag_finder
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltag_finder
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(apriltag_finder_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(apriltag_finder_generate_messages apriltag_finder_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv" NAME_WE)
add_dependencies(apriltag_finder_generate_messages_eus _apriltag_finder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_finder_geneus)
add_dependencies(apriltag_finder_geneus apriltag_finder_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_finder_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(apriltag_finder
  "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltag_finder
)

### Generating Module File
_generate_module_lisp(apriltag_finder
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltag_finder
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(apriltag_finder_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(apriltag_finder_generate_messages apriltag_finder_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv" NAME_WE)
add_dependencies(apriltag_finder_generate_messages_lisp _apriltag_finder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_finder_genlisp)
add_dependencies(apriltag_finder_genlisp apriltag_finder_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_finder_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(apriltag_finder
  "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltag_finder
)

### Generating Module File
_generate_module_nodejs(apriltag_finder
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltag_finder
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(apriltag_finder_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(apriltag_finder_generate_messages apriltag_finder_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv" NAME_WE)
add_dependencies(apriltag_finder_generate_messages_nodejs _apriltag_finder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_finder_gennodejs)
add_dependencies(apriltag_finder_gennodejs apriltag_finder_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_finder_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(apriltag_finder
  "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_finder
)

### Generating Module File
_generate_module_py(apriltag_finder
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_finder
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(apriltag_finder_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(apriltag_finder_generate_messages apriltag_finder_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/apriltag_finder/srv/ApriltagsIDsSrv.srv" NAME_WE)
add_dependencies(apriltag_finder_generate_messages_py _apriltag_finder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(apriltag_finder_genpy)
add_dependencies(apriltag_finder_genpy apriltag_finder_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS apriltag_finder_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltag_finder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/apriltag_finder
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(apriltag_finder_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(apriltag_finder_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltag_finder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/apriltag_finder
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(apriltag_finder_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(apriltag_finder_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltag_finder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/apriltag_finder
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(apriltag_finder_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(apriltag_finder_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltag_finder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/apriltag_finder
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(apriltag_finder_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(apriltag_finder_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_finder)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_finder\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/apriltag_finder
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(apriltag_finder_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(apriltag_finder_generate_messages_py geometry_msgs_generate_messages_py)
endif()
