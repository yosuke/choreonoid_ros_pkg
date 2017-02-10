#!/bin/bash
#
#

PROJECT_TEMPLATE_FILE="./test-contacts-state.cnoid.template"
MODEL_RELATIVE_PATH="\.\.\/\.\.\/choreonoid_ros\/model\/"

if [ ${#} -ne 5 ]; then
  echo -n "usage: "`basename ${0}`" <model filename> <model's base-link> <model position (vector3)>"
  echo " <model attitude (matrix3> <output project file>"

  exit 1
fi

if [ ! -f "${PROJECT_TEMPLATE_FILE}" ]; then
  echo "error: ${PROJECT_TEMPLATE_FILE} does not exists"
  exit 1
fi

cp ${PROJECT_TEMPLATE_FILE} ${5} || exit ${?}

_BODY_NAME=`echo ${2} | tr "[:upper:]" "[:lower:]"`
_CMD="sed -i -e 's/\%\%\%BODY_NAME\%\%\%/${_BODY_NAME}/' ${5}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_FILE\%\%\%/${MODEL_RELATIVE_PATH}${1}/' ${5}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_BASE_LINK\%\%\%/${2}/' ${5}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_ROOT_POSITION\%\%\%/${3}/' ${5}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_INITIAL_POSITION\%\%\%/${3}/' ${5}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_ROOT_ATTITUDE\%\%\%/${4}/' ${5}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_INITIAL_ATTITUDE\%\%\%/${4}/' ${5}"
eval ${_CMD} || exit ${?}

exit ${?}
