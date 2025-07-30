# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "assignment2_package: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(assignment2_package_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv" NAME_WE)
add_custom_target(_assignment2_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "assignment2_package" "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv" "geometry_msgs/PoseStamped:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv" NAME_WE)
add_custom_target(_assignment2_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "assignment2_package" "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv" "geometry_msgs/PoseStamped:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Pose"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignment2_package
)
_generate_srv_cpp(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignment2_package
)

### Generating Module File
_generate_module_cpp(assignment2_package
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignment2_package
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(assignment2_package_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(assignment2_package_generate_messages assignment2_package_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_cpp _assignment2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_cpp _assignment2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignment2_package_gencpp)
add_dependencies(assignment2_package_gencpp assignment2_package_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignment2_package_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignment2_package
)
_generate_srv_eus(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignment2_package
)

### Generating Module File
_generate_module_eus(assignment2_package
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignment2_package
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(assignment2_package_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(assignment2_package_generate_messages assignment2_package_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_eus _assignment2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_eus _assignment2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignment2_package_geneus)
add_dependencies(assignment2_package_geneus assignment2_package_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignment2_package_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignment2_package
)
_generate_srv_lisp(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignment2_package
)

### Generating Module File
_generate_module_lisp(assignment2_package
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignment2_package
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(assignment2_package_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(assignment2_package_generate_messages assignment2_package_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_lisp _assignment2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_lisp _assignment2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignment2_package_genlisp)
add_dependencies(assignment2_package_genlisp assignment2_package_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignment2_package_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignment2_package
)
_generate_srv_nodejs(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignment2_package
)

### Generating Module File
_generate_module_nodejs(assignment2_package
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignment2_package
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(assignment2_package_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(assignment2_package_generate_messages assignment2_package_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_nodejs _assignment2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_nodejs _assignment2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignment2_package_gennodejs)
add_dependencies(assignment2_package_gennodejs assignment2_package_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignment2_package_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignment2_package
)
_generate_srv_py(assignment2_package
  "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignment2_package
)

### Generating Module File
_generate_module_py(assignment2_package
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignment2_package
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(assignment2_package_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(assignment2_package_generate_messages assignment2_package_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/PickObject.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_py _assignment2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nikolavukas/project/src/assignment2_package/srv/GetObjectPose.srv" NAME_WE)
add_dependencies(assignment2_package_generate_messages_py _assignment2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(assignment2_package_genpy)
add_dependencies(assignment2_package_genpy assignment2_package_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS assignment2_package_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignment2_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/assignment2_package
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(assignment2_package_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(assignment2_package_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignment2_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/assignment2_package
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(assignment2_package_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(assignment2_package_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignment2_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/assignment2_package
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(assignment2_package_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(assignment2_package_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignment2_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/assignment2_package
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(assignment2_package_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(assignment2_package_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignment2_package)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignment2_package\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/assignment2_package
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(assignment2_package_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(assignment2_package_generate_messages_py geometry_msgs_generate_messages_py)
endif()
