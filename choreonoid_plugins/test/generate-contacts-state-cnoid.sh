#!/bin/bash
#
#

PROJECT_TEMPLATE_FILE="./test-contacts-state.cnoid.template"
MODEL_RELATIVE_PATH="\.\.\/\.\.\/choreonoid_ros\/model\/"

if [ ${#} -ne 4 ]; then
  echo "usage: "`basename ${0}`" <model filename> <model's base-link> <model position> <output project file>" 
  exit 1
fi

if [ ! -f "${PROJECT_TEMPLATE_FILE}" ]; then
  echo "error: ${PROJECT_TEMPLATE_FILE} does not exists"
  exit 1
fi

cp ${PROJECT_TEMPLATE_FILE} ${4} || exit ${?}

_BODY_NAME=`echo ${2} | tr "[:upper:]" "[:lower:]"`
_CMD="sed -i -e 's/\%\%\%BODY_NAME\%\%\%/${_BODY_NAME}/' ${4}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_FILE\%\%\%/${MODEL_RELATIVE_PATH}${1}/' ${4}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_BASE_LINK\%\%\%/${2}/' ${4}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_ROOT_POSITION\%\%\%/${3}/' ${4}"
eval ${_CMD} || exit ${?}
_CMD="sed -i -e 's/\%\%\%MODEL_INITIAL_POSITION\%\%\%/${3}/' ${4}"
eval ${_CMD} || exit ${?}

exit ${?}
