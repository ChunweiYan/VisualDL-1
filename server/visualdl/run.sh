#!/bin/bash
set -ex

WORKSPACE=$(pwd)/../..
BUILD_ROOT=$WORKSPACE/build

log_dir=$1
ip=$2

export PYTHONPATH="$(pwd)/..:${BUILD_ROOT}/visualdl/logic:${WORKSPACE}/visualdl/python"
export FLASK_APP=visual_dl.py
export FLASK_DEBUG=0

python visual_dl.py --logdir $log_dir --host $ip --port 8090
