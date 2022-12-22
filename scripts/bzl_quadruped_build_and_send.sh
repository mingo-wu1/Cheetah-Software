#!/bin/bash
set -e

QUADRUPED_IP="192.168.12.1"

script_abs=$(readlink -f "$0")
script_dir=$(dirname $script_abs)

cd $script_dir/../../
source /opt/ros/galactic/setup.bash

if [ $# -gt 0 ]; then
    colcon --log-base bzl_quadruped_log build --packages-select bzl_quadruped --build-base bzl_quadruped_build --install-base bzl_quadruped_install --cmake-args -DMINI_CHEETAH_BUILD=ON $*
else
    colcon --log-base bzl_quadruped_log build --packages-select bzl_quadruped --build-base bzl_quadruped_build --install-base bzl_quadruped_install --cmake-args -DMINI_CHEETAH_BUILD=ON
fi

if [ $? -eq 0 ]; then
    rsync -azP bzl_quadruped_install user@${QUADRUPED_IP}:~/
    echo "succeed"
else
    echo "failed"
fi

