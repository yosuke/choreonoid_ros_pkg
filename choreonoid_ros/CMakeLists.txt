# http://ros.org/doc/hydro/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(choreonoid_ros)

find_package(catkin REQUIRED COMPONENTS mk)

catkin_package(DEPENDS eigen)

execute_process(
    COMMAND
    ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_BINARY_DIR}
    make -f ${PROJECT_SOURCE_DIR}/Makefile.choreonoid INSTALL_DIR=${CATKIN_DEVEL_PREFIX} installed.choreonoid
    RESULT_VARIABLE
    _make_failed
)
if (_make_failed)
  message(FATAL_ERROR "Failed to build choreonoid: ${_make_failed}")
endif(_make_failed)

add_custom_target(clean
   COMMAND
   ${CMAKE_COMMAND} -E chdir ${CMAKE_CURRENT_BINARY_DIR}
   make -f ${PROJECT_SOURCE_DIR}/Makefile.choreonoid INSTALL_DIR=${CATKIN_DEVEL_PREFIX} clean.choreonoid
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
