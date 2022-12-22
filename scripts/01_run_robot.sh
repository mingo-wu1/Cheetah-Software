#!/bin/bash
set -e

script_abs=$(readlink -f "$0")
script_dir=$(dirname $script_abs)

sudo su << EOF
    cd $script_dir
    source ../../setup.bash
    export ROS_LOG_DIR=/home/user/.bzl_quadruped_log/
    if [ $# -gt 0 ]; then
        bzl_ctrl $*
    else
        bzl_ctrl m r f
    fi
EOF
