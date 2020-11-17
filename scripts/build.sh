#!/bin/bash

source /opt/ros/kinetic/setup.bash
catkin_make --pkg rvio --cmake-args -DCMAKE_BUILD_TYPE=Release
