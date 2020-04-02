#!/usr/bin/env bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$DIR/models