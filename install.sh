#!/bin/bash

if [ `id -u` == 0 ]; then
    SUDO=
    export DEBIAN_FRONTEND=noninteractive
else
    SUDO="sudo -H"
fi

source /opt/ros/${ROS_DISTRO}/setup.bash
${SUDO} apt update
${SUDO} apt install -y python3-wstool python3-catkin-tools python3-rosdep
wstool init .
wstool merge -t . ./packages.rosinstall
wstool update -t .
rosdep update && rosdep install -r -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y
