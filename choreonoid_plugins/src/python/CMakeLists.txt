# @author 

set(target PyRosPlugin)
string(REGEX REPLACE "^Py(.+)$" "\\1" module ${target})

add_cnoid_python_module(${target}
  PyRosPluginModule.cpp
  PyItems.cpp
)

target_link_libraries(${target} CnoidRosBodyPlugin CnoidPyBase)

install(TARGETS 
  ${target}
  RUNTIME DESTINATION ${CNOID_PYTHON_SUBDIR}/cnoid
  LIBRARY DESTINATION ${CNOID_PYTHON_SUBDIR}/cnoid
  ARCHIVE DESTINATION ${CNOID_PYTHON_SUBDIR}/cnoid
)
