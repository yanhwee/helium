#!/usr/bin/env bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

export GAZEBO_MODEL_PATH=$DIR/models:${GAZEBO_MODEL_PATH}

export GAZEBO_RESOURCE_PATH=$DIR/worlds:${GAZEBO_RESOURCE_PATH}