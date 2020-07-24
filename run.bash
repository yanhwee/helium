#!/usr/bin/env bash
set -e
set -x

start() {
    cd $HOME
    gnome-terminal --tab -- bash -c "roslaunch helium hills_pillars_multi.launch"
    gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --out=tcpin:0.0.0.0:8100"
    gnome-terminal --tab -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris-2 -I1 --out=tcpin:0.0.0.0:8200"
    gnome-terminal --tab -- bash -c "trap exit SIGINT; while true; do roslaunch helium apm.launch; done"
    gnome-terminal --tab -- bash -c "trap exit SIGINT; while true; do roslaunch helium apm2.launch; done"
}

list() {
    pgrep -f 'roslaunch helium'
    pgrep -f 'sim_vehicle.py'
}

stop() {
    if    [ "$(list)" ]; then kill -INT $(list); fi
    while [ "$(list)" ]; do sleep 1;             done
    exit
}

if [ "$(list)" ]; then exit; fi

trap stop SIGINT

start

sleep infinity