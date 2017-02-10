#!/bin/bash
#
#

_CMD="./generate-contacts-state-cnoid.sh"

_ARG_TEST00="box00.wrl BOX00 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box00.cnoid"
_ARG_TEST01="box01.wrl BOX01 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box01.cnoid"
_ARG_TEST02="box02.wrl BOX02 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box02.cnoid"
_ARG_TEST03="box03.wrl BOX03 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box03.cnoid"
_ARG_TEST04="box04.wrl BOX04 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box04.cnoid"
_ARG_TEST05="box05.wrl BOX05 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box05.cnoid"
_ARG_TEST06="box06.wrl BOX06 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box06.cnoid"
_ARG_TEST07="box07.wrl BOX07 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box07.cnoid"
_ARG_TEST08="box08.wrl BOX08 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box08.cnoid"
_ARG_TEST09="box09.wrl BOX09 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box09.cnoid"
_ARG_TEST10="box10.wrl BOX10 \"0.0, 0.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box10.cnoid"
_ARG_TEST21="box00.wrl BOX00 \"1.0, 1.0, 20.0\"   \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box21.cnoid"
_ARG_TEST22="box00.wrl BOX00 \"-1.0, -1.0, 20.0\" \"1, 0, 0, 0, 1, 0, 0, 0, 1\" test-cs-box22.cnoid"
# R=45.0, P=0.0, Y=0.0 [deg]
_ARG_TEST31="box00.wrl BOX00 \"0.5, 0.5, 20.0\" \"1, 0, 0, 0, 0.707106781, -0.707106781, 0, 0.707106781, 0.707106781\" test-cs-box31.cnoid"
# R=45.0, P=45.0, Y=0.0 [deg]
_ARG_TEST32="box00.wrl BOX00 \"0.5, 0.5, 20.0\" \"0.707106781, 0.5, 0.5, 0, 0.707106781, -0.707106781, -0.707106781, 0.5, 0.5\" test-cs-box32.cnoid"
# R=90.0, P=0.0, Y=0.0 [deg]
_ARG_TEST33="box00.wrl BOX00 \"0.5, 0.5, 20.0\" \"1, 0, 0, 0, 0, -1, 0, 1, 0\" test-cs-box33.cnoid"
# R=180.0, P=0.0, Y=0.0 [deg]
_ARG_TEST34="box00.wrl BOX00 \"0.5, 0.5, 20.0\" \"1, 0, 0, 0, -1, -0, 0, 0, -1\" test-cs-box34.cnoid"
# R=-90.0, P=0.0, Y=0.0 [deg]
_ARG_TEST35="box00.wrl BOX00 \"0.5, 0.5, 20.0\" \"1, 0, 0, 0, 0, 1, 0, -1, 0\" test-cs-box35.cnoid"

eval ${_CMD} ${_ARG_TEST00} || exit ${?}
eval ${_CMD} ${_ARG_TEST01} || exit ${?}
eval ${_CMD} ${_ARG_TEST02} || exit ${?}
eval ${_CMD} ${_ARG_TEST03} || exit ${?}
eval ${_CMD} ${_ARG_TEST04} || exit ${?}
eval ${_CMD} ${_ARG_TEST05} || exit ${?}
eval ${_CMD} ${_ARG_TEST06} || exit ${?}
eval ${_CMD} ${_ARG_TEST07} || exit ${?}
eval ${_CMD} ${_ARG_TEST08} || exit ${?}
eval ${_CMD} ${_ARG_TEST09} || exit ${?}
eval ${_CMD} ${_ARG_TEST10} || exit ${?}
eval ${_CMD} ${_ARG_TEST21} || exit ${?}
eval ${_CMD} ${_ARG_TEST22} || exit ${?}
eval ${_CMD} ${_ARG_TEST31} || exit ${?}
eval ${_CMD} ${_ARG_TEST32} || exit ${?}
eval ${_CMD} ${_ARG_TEST33} || exit ${?}
eval ${_CMD} ${_ARG_TEST34} || exit ${?}
eval ${_CMD} ${_ARG_TEST35} || exit ${?}

exit ${?}
