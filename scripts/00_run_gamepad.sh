#!/bin/bash
set -e

script_abs=$(readlink -f "$0")
script_dir=$(dirname $script_abs)

sudo su << EOF
    cd $script_dir
    source ../../setup.bash
    gamepad
EOF