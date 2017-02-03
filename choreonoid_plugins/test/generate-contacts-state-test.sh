#!/bin/bash
#
#

_CMD="./generate-contacts-state-cnoid.sh"

_ARG_TEST00="box00.wrl BOX00 \"0.0, 0.0, 20.0\" test-cs-box00.cnoid"
_ARG_TEST01="box01.wrl BOX01 \"0.0, 0.0, 20.0\" test-cs-box01.cnoid"
_ARG_TEST02="box02.wrl BOX02 \"0.0, 0.0, 20.0\" test-cs-box02.cnoid"
_ARG_TEST03="box03.wrl BOX03 \"0.0, 0.0, 20.0\" test-cs-box03.cnoid"
_ARG_TEST04="box04.wrl BOX04 \"0.0, 0.0, 20.0\" test-cs-box04.cnoid"
_ARG_TEST05="box05.wrl BOX05 \"0.0, 0.0, 20.0\" test-cs-box05.cnoid"
_ARG_TEST06="box06.wrl BOX06 \"0.0, 0.0, 20.0\" test-cs-box06.cnoid"
_ARG_TEST07="box07.wrl BOX07 \"0.0, 0.0, 20.0\" test-cs-box07.cnoid"
_ARG_TEST08="box08.wrl BOX08 \"0.0, 0.0, 20.0\" test-cs-box08.cnoid"
_ARG_TEST09="box09.wrl BOX09 \"0.0, 0.0, 20.0\" test-cs-box09.cnoid"
_ARG_TEST10="box10.wrl BOX10 \"0.0, 0.0, 20.0\" test-cs-box10.cnoid"

_ARG_TEST21="box00.wrl BOX00 \"1.0, 1.0, 20.0\" test-cs-box21.cnoid"
_ARG_TEST22="box00.wrl BOX00 \"-1.0, -1.0, 20.0\" test-cs-box22.cnoid"

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

exit ${?}
