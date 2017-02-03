#!/bin/bash
#
# Do testing feature of publish contacts state.
# This script purpose is to make the test simple.
#

PACKAGE_NAME="choreonoid_ros"
LAUNCH_CMD="roslaunch"
LAUNCH_FILE=""
LAUNCH_ARGS=""
TEST_NO=0

#
# Function.
#

function usage()
{
  echo `basename ${0}`" <choreonoid|gazebo> <test No.>"
  exit 1
}

function generate_choreonoid_args()
{
  LAUNCH_ARGS="cnoid_file:=test-cs-box${TEST_NO}.cnoid"
}

function generate_gazebo_args()
{
  case "${TEST_NO}" in
    21)
      LAUNCH_ARGS="identity:=box${TEST_NO} urdf_file:=box00.urdf model_xpos:=1.0 model_ypos:=1.0"
      ;;

    22)
      LAUNCH_ARGS="identity:=box${TEST_NO} urdf_file:=box00.urdf model_xpos:=-1.0 model_ypos:=-1.0"
      ;;

    *)
      LAUNCH_ARGS="identity:=box${TEST_NO} urdf_file:=box${TEST_NO}.urdf"
      ;;
  esac
}

#
# Main.
#

if [ ${#} -lt 2 ]; then
  usage
fi

TEST_NO=`printf "%02d" ${2}`

case "${1}" in
  choreonoid)
    LAUNCH_FILE="test-cnoidros-contacts-state.launch"
    generate_choreonoid_args
    ;;

  gazebo)
    LAUNCH_FILE="test-gzros-contacts-state.launch"
    generate_gazebo_args
    ;;

  *)
    usage
    ;;
esac

${LAUNCH_CMD} ${PACKAGE_NAME} ${LAUNCH_FILE} ${LAUNCH_ARGS}

exit ${?}
