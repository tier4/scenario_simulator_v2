#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

touch $XAUTH

xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm \
 -e XAUTHORITY=${XAUTH} \
 -e DISPLAY=$DISPLAY \
 -e QT_GRAPHICSSYSTEM=native \
 -v $XSOCK:$XSOCK:rw \
 -v $XAUTH:$XAUTH:rw \
 tier4/scenario_simulator_v2 \
 bash