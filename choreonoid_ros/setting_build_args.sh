#!/bin/sh
# This script is setting CMake arguments for build of the Choreonoid.
#

#
# Variables.
#

# for cmake
_CNOID_ROS_PKG_BUILD_CNOID_USER_CMAKE_ARGS=""
_CNOID_ROS_PKG_BUILD_CNOID_CONSTANT_CMAKE_ARGS="
  -DENABLE_INSTALL_RPATH=ON
  -DENABLE_PYTHON=OFF
  -DBUILD_PYTHON_PLUGIN=OFF
  -DBUILD_PYTHON_SIM_SCRIPT_PLUGIN=OFF
  -DBUILD_AGX_DYNAMICS_PLUGIN=ON
  -DBUILD_AGX_BODYEXTENSION_PLUGIN=ON
  -DAGX_DIR=/opt/Algoryx/AgX-2.24.0.1
  -DBUILD_SCENE_EFFECTS_PLUGIN=ON
  -DBUILD_TRAFFIC_CONTROL_PLUGIN=ON
  -DBUILD_MULTICOPTER_PLUGIN=ON
  -DBUILD_MULTICOPTER_SAMPLES=ON
  -DUSE_PYTHON3=OFF
  "

# for make
_CNOID_ROS_PKG_BUILD_CNOID_MAKE_ARGS="-j$(nproc)"

#
# Main.
#

_dir=`dirname ${0}`
_fname=""

case ${1} in
  cmake)
    _fname="${_dir}/additional_cmake_args"

    if [ -f ${_fname} ]; then
      _CNOID_ROS_PKG_BUILD_CNOID_USER_CMAKE_ARGS=`cat ${_fname}`
    fi

    echo ${_CNOID_ROS_PKG_BUILD_CNOID_CONSTANT_CMAKE_ARGS} ${_CNOID_ROS_PKG_BUILD_CNOID_USER_CMAKE_ARGS}
    ;;

  make)
    _fname="${_dir}/additional_make_args"

    if [ -f ${_fname} ]; then
      _CNOID_ROS_PKG_BUILD_CNOID_MAKE_ARGS=`cat ${_fname}`
    fi

    echo ${_CNOID_ROS_PKG_BUILD_CNOID_MAKE_ARGS}
    ;;

  *)
    echo "usage:" `basename ${0}` "[ cmake | make ]"
    exit 1
    ;;
esac

exit 0
